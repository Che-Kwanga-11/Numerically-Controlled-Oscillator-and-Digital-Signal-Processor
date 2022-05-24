// UART0 Library
// Che Kwanga

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include<stdlib.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"

// PortA masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART0
void initUart0()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    _delay_cycles(3);

    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= UART_TX_MASK;                   // enable output on UART0 TX pin
    GPIO_PORTA_DIR_R &= ~UART_RX_MASK;                   // enable input on UART0 RX pin
    GPIO_PORTA_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART0 pins
    GPIO_PORTA_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA1_M | GPIO_PCTL_PA0_M); // clear bits 0-7
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                        // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART0_IBRD_R = 21;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                  // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // enable TX, RX, and module
}

// Set baud rate as function of instruction cycle frequency
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    UART0_IBRD_R = divisorTimes128 >> 7;                 // set integer value to floor(r)
    UART0_FBRD_R = ((divisorTimes128 + 1)) >> 1 & 63;    // set fractional value to round(fract(r)*64)
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

// Returns the status of the receive buffer
bool kbhitUart0()
{
    return !(UART0_FR_R & UART_FR_RXFE);
}

void parseFields(USER_DATA* data){
    int fieldCount=0;
    int i=0;
    char c;
    bool delimiter; //keeps the character's last field value i.e delimiter or not
    delimiter=true;
    c=data->buffer[i];
    while(c!='\0'){
    if(((97<=c)&&(c<=122))||((101<=c)&&(c<=132))){
                 if(delimiter==true){
                  delimiter=false;
                  data->fieldPosition[fieldCount]=i;
                  data->fieldType[fieldCount]='a';
                  fieldCount++;
                 }
             }else if(((48<=c)&&(c<=57))||((45<=c)&&(c<=46))){
                 if(delimiter==true){
                 delimiter=false;
                 data->fieldPosition[fieldCount]=i;
                 data->fieldType[fieldCount]='n';
                 fieldCount++;
                 }}else{
                     delimiter=true;
                     data->buffer[i]='\0';
                 }
    i++;
    c=data->buffer[i];
    }
   data->fieldCount=fieldCount;
    return;
}

char* getFieldString(USER_DATA* data, uint8_t fieldNumber){
  if(fieldNumber<data->fieldCount){
      return (char*)(data->buffer[data->fieldPosition[fieldNumber]]);
  }
  return '\0';
}
void getsUart0(USER_DATA* data){
    int count=0;

    char c;
    while (count<=MAX_CHARS){
        c=getcUart0();
        putcUart0(c);
        if(c==127){
            if(count>0){
                count--;
            } //backspace key
        }else if(c==13){
            data->buffer[count]='\0';
            count++;
            return; //carriage return
        }else if(c>=32){
            data->buffer[count]=c;
            count++;
        } // characters
    }
    data->buffer[count]='\0';
    return;
}
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber){
    if(fieldNumber<data->fieldCount){
        if(data->fieldType[fieldNumber]=='n'){
          return (int32_t)data->buffer[data->fieldPosition[fieldNumber]];
      }
    return 0;
    }
    return 0;
}
double getFieldDouble(USER_DATA* dataPtr, uint8_t fieldNumber){
    USER_DATA data=*dataPtr;
    if(fieldNumber<data.fieldCount){
        if(data.fieldType[fieldNumber]=='n'){
          return (double)strtod(((char*)(&data.buffer[data.fieldPosition[fieldNumber]])),NULL);
      }
    return 0;
    }
    return 0;
}

bool isCommand(USER_DATA* data, const char strCommand[],uint8_t minArguments){
        USER_DATA rawData=*data;
        int commandPosition=rawData.fieldPosition[0];
        int i=0;
        char command[80]="";
        while(rawData.buffer[commandPosition]!='\0'){
            command[i++]=rawData.buffer[commandPosition++];
        }
    if(sameString(command,strCommand)){
    if(((data->fieldCount)-1)>=minArguments){ // -1 for the command field
        return 1;
    }
    }

   return 0;
}
bool sameString(char str[], const char stm[]){
     uint8_t i=0;
     while(str[i]==stm[i]){
         i++;
         if(str[i]=='\0'){
             return 1;
         }
     }
     return 0;
 }
