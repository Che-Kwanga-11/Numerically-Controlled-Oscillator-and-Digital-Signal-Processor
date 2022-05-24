// Embedded Project
// Che Kwanga

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// SPI1 Interface:
//   MOSI on PD3 (SSI1Tx)
//   MISO on PD2 (SSI1Rx)
//   ~CS on PD1  (SSI1Fss)
//   SCLK on PD0 (SSI1Clk)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "tm4c123gh6pm.h"
#include "spi1.h"
#include "gpio.h"
#include "clock.h"
#include "wait.h"
#include "nvic.h"
#include "uart0.h"
#include "adc0.h"

#define SSI1TX PORTD,3
#define SSI1RX PORTD,2
#define SSI1FSS PORTD,1
#define SSI1CLK PORTD,0
#define PI 3.142857
#define lutSize 2048
#define freqSize 30
#define twoToTheThreeTwo 4294967296
#define dacA 0x3000
#define dacB 0xB000
#define gainA ((0X3B90-0X3420)/-5)
#define gainB ((0X3B90-0X3420)/-5)
#define Ra 0x7D3  // offset of 0V in R
#define Rb 0x7D3  // offset of 0V in R // 0x3B90 -2.5 ON A  0x3420 +2.5v ON A
#define AIN1_MASK 4
#define AIN0_MASK 8

USER_DATA data;
int32_t A=(int32_t) '1'; // assigned channel A also assigned to input 1 that is PE3 readAdcSs2 for control voltage
int32_t B=(int32_t) '2'; // assigned to channel B also assigned to input 2 that is PE2 readAdcSs3

uint32_t l=0;
uint32_t cycles;
uint16_t lutA[lutSize];
uint16_t lutB[lutSize];
//uint32_t frequencies[freqSize];
uint32_t delta_phaseA;
uint32_t delta_phaseB;
uint32_t phase_accumulatorA=0;
uint32_t phase_accumulatorB=0;
uint32_t fs=100000;
double previousAmp;
double previousOfs;
double scalingFactor;
char str[100];
uint16_t raw;
float final;
// flags
bool setCycles=false;
bool timerSet=false;
bool displayVoltage=false;
bool levelOn=false;
bool differential=false;

void spiDACWrite(uint32_t input){
    setPinValue(SSI1FSS,0);
    _delay_cycles(1);
    writeSpi1Data(input);    // register
    readSpi1Data();
    setPinValue(SSI1FSS,1);
    _delay_cycles(1);
    setPinValue(PORTE,1,0);
    _delay_cycles(4);
    setPinValue(PORTE,1,1);
    _delay_cycles(1);
}
void dacInit(){
    enablePort(PORTD);
    enablePort(PORTE);
    // Initialize SPI1 interface
    initSpi1(USE_SSI_FSS);               // to receive and send data
    setSpi1BaudRate(20e6, 40e6);        // set baud rate
    setSpi1Mode(0, 0);                 // expander uses 1,1 or 0,0

    selectPinPushPullOutput(SSI1FSS);  //set SSI1FSS as an output
    selectPinDigitalInput(PORTE,1);

    setPinValue(SSI1FSS,1);
    setPinValue(SSI1FSS,0);
    setPinValue(SSI1FSS,1);

    setPinValue(PORTE,1,1);
    setPinValue(PORTE,1,0);
    setPinValue(PORTE,1,1);

    spiDACWrite(dacA+(Ra+gainA*0));        // set to 0V
    spiDACWrite(dacB+(Rb+gainB*0));        // set to 0V
}
void initAnalog()
{
    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
    _delay_cycles(3);

    // Configure AIN1 and AIN0 as an analog inputs
    GPIO_PORTE_AFSEL_R |= AIN1_MASK | AIN0_MASK;                 // select alternative functions for AN3 (PE0)
    GPIO_PORTE_DEN_R &= ~AIN1_MASK| AIN0_MASK;                  // turn off digital operation on pin PE0
    GPIO_PORTE_AMSEL_R |= AIN1_MASK| AIN0_MASK;                 // turn on analog operation on pin PE0
}
void setTimer(){
        // Enable clocks
        SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
        _delay_cycles(3);

        // Configure Timer 1 as the time base
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
        TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
        TIMER1_TAILR_R = 400;                            // sampling freq=40Mhz/(TAILR-1)=  KHz sampling rate
        TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
        TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
        NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A) in NVIC
        timerSet=true;
}
 void stop(){
     if(timerSet){
     TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
     }
     timerSet=false;
     displayVoltage=false;
 }
 void diff(USER_DATA* data){
     double choice=getFieldDouble(data,1);
          if(choice==1.0){
              differential=true;
          }
          if(choice==0.0){
              differential=false;
          }
  }
 void reset(){
      // NVIC_APINT_R= NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ; // sends you to resetISR
     stop();                                                        // stop all waves first
     __asm("    .global _c_int00\n"
           "    b.w     _c_int00");
     spiDACWrite(dacA+(Ra+gainA*0));
     spiDACWrite(dacB+(Rb+gainB*0));
  }
 void dc(USER_DATA* data){
     int32_t channel=getFieldInteger(data,1);
     double output=getFieldDouble(data,2);
     if(channel==A){
         putsUart0("\nOutput is DC on channel A\n");
         spiDACWrite(dacA+(Ra+gainA*output));
     }
     if(channel==B){
         putsUart0("\nOutput is DC on channel B\n");
         spiDACWrite(dacB+(Rb+gainB*output));
     }
 }
 void level(USER_DATA* data){
     int i=0;
     double levelChoice=getFieldDouble(data,1);
     double voltageA, voltageB;
     if(levelChoice==1.0){
         levelOn=true;
         voltageA=readAdc0Ss2();
         voltageA=((float)((voltageA*0.0008)));
         voltageB=readAdc0Ss3();
         voltageB=((float)((voltageB*0.0008)));
         scalingFactor=((float)(voltageA/voltageB));
         sprintf(str, "Scaling Factor:      %0.4f\n", scalingFactor);
         putsUart0(str);
         // then adjust lutB since it is the test input.
         if(timerSet){
              TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
              }
         for(i=0;i<lutSize;i++){
             lutB[i]-=dacB;
             lutB[i]-=Rb;
             lutB[i]-=(float)(previousOfs*gainB);
             lutB[i]=(float)(lutB[i]*scalingFactor);
             lutB[i]+=(float)(previousOfs*gainB*scalingFactor);
             lutB[i]+=Rb;
             lutB[i]+=dacB;
         }
         if(timerSet){
             TIMER1_CTL_R |= TIMER_CTL_TAEN;
         }
     }
     if(levelChoice==0.0){
         levelOn=false;
         if(timerSet){
           TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
               }
         for(i=0;i<lutSize;i++){
           lutB[i]-=dacB;
           lutB[i]-=Rb;
           lutB[i]-=(float)(previousOfs*gainB*scalingFactor);
           lutB[i]=(float)(lutB[i]/scalingFactor);
           lutB[i]+=(float)(previousOfs*gainB);
           lutB[i]+=Rb;
           lutB[i]+=dacB;
          }
           if(timerSet){
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
            }
     }
 }
 void runCycles(USER_DATA* data){
      setCycles=true;
      cycles=getFieldDouble(data,1);
      cycles=cycles*2;                                  // times two account for both channels going through the same number of cycles
  }
 void sine(USER_DATA* data){
     int i=0;
     int32_t channel=getFieldInteger(data,1);
     double freq=getFieldDouble(data,2);
     double amp=getFieldDouble(data,3);
     double offset=getFieldDouble(data,4);
     double phaseShift=getFieldDouble(data,5);
     previousAmp=amp;
     previousOfs=offset;
     float slice;
     if(channel==A){
       putsUart0("\nOutput is A\n");
     for(i=0;i<lutSize;i++){
             slice=(float)i/lutSize;
             lutA[i]=Ra+((float)(offset*gainA))+((float)((gainA*amp)*(sin(2*PI*slice)))); //             lutA[i]=Ra+((float)(offset*gainA))+((float)((gainA*amp)*(sin(2*PI*slice+(phaseShift*PI/180))));
             lutA[i]+=dacA;
             if(differential){
                 lutB[i]=Rb+((float)(offset*gainB))+((float)((gainB*amp*-1)*(sin(2*PI*slice))));  // lutB[i]=Rb+((float)(offset*gainB))+((float)((gainB*amp)*(sin(2*PI*slice+(phaseShift*PI/180)))));
                 lutB[i]+=dacB;
                 delta_phaseB=(float)(twoToTheThreeTwo*(((float)(freq))/((float)(fs))));       // fs- sampling rate divide frequency by 2
             }
     }
     delta_phaseA=(float)(twoToTheThreeTwo*(((float)(freq))/((float)(fs))));       // fs- sampling rate divide frequency by 2
     }
     if(channel==B){
            putsUart0("\nOutput is B\n");
          for(i=0;i<lutSize;i++){
                  slice=(float)i/lutSize;
                  lutB[i]=Rb+((float)(offset*gainB))+((float)((gainB*amp)*(sin(2*PI*slice))));  // lutB[i]=Rb+((float)(offset*gainB))+((float)((gainB*amp)*(sin(2*PI*slice+(phaseShift*PI/180)))));
                  lutB[i]+=dacB;
          }
      delta_phaseB=(float)(twoToTheThreeTwo*(((float)(freq))/((float)(fs))));       // fs- sampling rate divide frequency by 2
          }

 }
 void square(USER_DATA* data){
      int i=0;
      int32_t channel=getFieldInteger(data,1);
      uint32_t freq=getFieldDouble(data,2);
      double amp=getFieldDouble(data,3);
      double offset=getFieldDouble(data,4);
      double dutyCycle=getFieldDouble(data,5);
      dutyCycle=dutyCycle/100;
      uint32_t limiter=(uint32_t)(lutSize*dutyCycle);
      previousAmp=amp;                            // for level command
      previousOfs=offset;                         // for level command
      if(channel==A){
        putsUart0("\nOutput is A\n");
      for(i=0;i<lutSize;i++){
          if(i<(limiter)){
              lutA[i]=Ra+((float)(offset*gainA))+((float)(gainA*amp));
              lutA[i]+=dacA;
          if(differential){
               lutB[i]=Rb+((float)(offset*gainB))+((float)((gainB*amp*-1)));
               lutB[i]+=dacB;
                           }
      }else{
          lutA[i]=Ra+((float)(offset*gainA))+((float)(gainA*amp*0));
          lutA[i]+=dacA;
          if(differential){
            lutB[i]=Rb+((float)(offset*gainB))+((float)((gainB*amp*0)));
            lutB[i]+=dacB;
         }
      }
      }
      delta_phaseA=(float)(twoToTheThreeTwo*(((float)(freq))/((float)(fs))));       // fs- sampling rate divide frequency by 2
      }
      if(channel==B){
              putsUart0("\nOutput is B\n");
            for(i=0;i<lutSize;i++){
                if(i<(limiter)){
                    lutB[i]=Rb+((float)(offset*gainB))+((float)(gainB*amp));
                    lutB[i]+=dacB;
            }else{
               lutB[i]=Rb+((float)(offset*gainB))+((float)(gainB*amp*0));
               lutB[i]+=dacB;
            }
            }
            delta_phaseB=(float)(twoToTheThreeTwo*(((float)(freq))/((float)(fs))));       // fs- sampling rate divide frequency by 2
            }

  }
 void saw(USER_DATA* data){
      int i=0;
      int32_t channel=getFieldInteger(data,1);
      uint32_t freq=getFieldDouble(data,2);
      double amp=getFieldDouble(data,3);
      double offset=getFieldDouble(data,4);
      previousAmp=amp;                            // for level command
      previousOfs=offset;                         // for level command
      float slice;
      if(channel==A){
        putsUart0("\nOutput is A\n");
      for(i=0;i<lutSize;i++){
          slice=(float)i/lutSize;
         if(i<(lutSize/2)){
          lutA[i]=Ra+((float)(offset*gainA))+((float)((gainA*amp*0.5)*slice));
          lutA[i]+=dacA;
      }else{
          lutA[i]=Ra+((float)(offset*gainA))+((float)((gainA*amp*0.5)*slice));
          lutA[i]+=dacA;
      }}
      delta_phaseA=(float)(twoToTheThreeTwo*(((float)(freq))/((float)(fs))));       // fs- sampling rate divide frequency by 2
      }
      // channel B
      if(channel==B){
              putsUart0("\nOutput is B\n");
      for(i=0;i<lutSize;i++){
          slice=(float)i/lutSize;
          if(i<(lutSize/2)){
          lutB[i]=Rb+((float)(offset*gainB))+((float)((gainB*amp*0.5)*slice));
          lutB[i]+=dacB;
      }else{
          lutB[i]=Rb+((float)(offset*gainA))+((float)((gainB*amp*0.5)*slice));
          lutB[i]+=dacB;
       }}
      delta_phaseA=(float)(twoToTheThreeTwo*(((float)(freq))/((float)(fs))));       // fs- sampling rate divide frequency by 2
      }
  }
 void tri(USER_DATA* data){
     int i=0;
     int t=0;
     int32_t channel=getFieldInteger(data,1);
     uint32_t freq=getFieldDouble(data,2);
     double amp=getFieldDouble(data,3);
     double offset=getFieldDouble(data,4);
     previousAmp=amp;                            // for level command
     previousOfs=offset;                         // for level command
     float slice;
     if(channel==A){
       putsUart0("\nOutput is A\n");
       for(i=0;i<lutSize;i++){
                 slice=(float)i/lutSize;
                 if(i<1024){
                     lutA[i]=Ra+((float)(gainA*amp*slice*2));
                     lutA[i]+=dacA;
             }else{
                 t=2048-i;
                 slice=(float)t/lutSize;
                 lutA[i]=Ra+((float)(gainA*amp*slice*2));
                 lutA[i]+=dacA;
             }
             }
       delta_phaseA=(float)(twoToTheThreeTwo*(((float)(freq))/((float)(fs))));       // fs- sampling rate divide frequency by 2
     }
     if(channel==B){
           putsUart0("\nOutput is B\n");
           for(i=0;i<lutSize;i++){
                     slice=(float)i/lutSize;
                     if(i<1024){
                         lutB[i]=Rb+((float)(gainB*amp*slice*2));
                         lutB[i]+=dacB;
                 }else{
                     t=2048-i;
                     slice=(float)t/lutSize;
                     lutB[i]=Rb+((float)(gainB*amp*slice*2));
                     lutB[i]+=dacB;
                 }
                 }
      delta_phaseA=(float)(twoToTheThreeTwo*(((float)(freq))/((float)(fs))));       // fs- sampling rate divide frequency by 2
     }
 }
 void gain(USER_DATA* data){
     double freq1=getFieldDouble(data,1);
     double freq2=getFieldDouble(data,2);
     double control,test,gain;
     uint32_t stepSize=(freq2-freq1)/freqSize;
     uint32_t freq=freq1;
     int i=0;
     for(i=0;i<(freqSize+1);i++){
        if(timerSet){
             TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
             }
        delta_phaseA=(float)(twoToTheThreeTwo*(((float)(freq))/((float)(fs))));
        delta_phaseB=(float)(twoToTheThreeTwo*(((float)(freq))/((float)(fs))));
        if(timerSet){
             TIMER1_CTL_R |= TIMER_CTL_TAEN;
             }
        waitMicrosecond(100000);
        control=((float)((readAdc0Ss2()*0.0008)));
        test=((float)((readAdc0Ss3()*0.0008)));
        gain=((float)(20*log10(test/control)));
        sprintf(str, "%0.4f",gain);
        putsUart0(str);
        freq=freq+stepSize;
     }
     freq=freq1;
     for(i=0;i<freqSize+1;i++){
         sprintf(str, "%0.4u",freq);
         putsUart0(str);
         freq=freq+stepSize;
     }
 }
 void timer1Isr(){
     // channel A
         phase_accumulatorA=phase_accumulatorA+delta_phaseA; // phase_accumulator+delta_phase;
         if((phase_accumulatorA>>20)>= lutSize)
              {
                  phase_accumulatorA = 0;
                  cycles--;
              }
       spiDACWrite(lutA[(phase_accumulatorA>>20)]);
    // channel B
       phase_accumulatorB=phase_accumulatorB+delta_phaseB; // phase_accumulator+delta_phase;
       if((phase_accumulatorB>>20)>= lutSize)
           {
             phase_accumulatorB = 0;
             cycles--;
           }
       spiDACWrite(lutB[(phase_accumulatorB>>20)]);
    // cycles stop
       if((cycles==0)&&setCycles){
             stop();
        }
       TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
//     *if(l==(lutSize-1)){
//         l=0; // display lut at low frequency
//     }
//     spiDACWrite(lutA[l++]);
 }
int main(void) {
        // Initialize system clock to 40 MHz
        initSystemClockTo40Mhz();
        initUart0();
        dacInit();
        initAnalog();
        initAdc0Ss3();
        initAdc0Ss2();
        setAdc0Ss3Mux(1);   // 1 for AIN1 PE2
        setAdc0Ss3Log2AverageCount(6);
        setAdc0Ss2Mux(0);   // 0 for AIN0 PE3
        setAdc0Ss2Log2AverageCount(6);
        putsUart0("\nInitialization Done\n");
        while(1){
         if(kbhitUart0()){
         getsUart0(&data);
         putsUart0(data.buffer);
         putsUart0("\n");
         parseFields(&data);
         if(isCommand(&data,"dc",0)){dc(&data);}
         if(isCommand(&data,"sine",0)){sine(&data);}
         if(isCommand(&data,"square",0)){square(&data);}
         if(isCommand(&data,"sawtooth",0)){saw(&data);}
         if(isCommand(&data,"triangle",0)){tri(&data);}
         if(isCommand(&data,"run",0)){setTimer();}
         if(isCommand(&data,"stop",0)){stop();}
         if(isCommand(&data,"cycles",0)){runCycles(&data);}
         if(isCommand(&data,"reset",0)){reset();}
         if(isCommand(&data,"level",0)){level(&data);}
         if(isCommand(&data,"gain",0)){gain(&data);}
         if(isCommand(&data,"differential",0)){diff(&data);} // have to toggle it first
         if(isCommand(&data,"voltage",0)||displayVoltage){
              displayVoltage=true;
            int32_t channel=getFieldInteger(&data,1);
            while(displayVoltage&&!(kbhitUart0())){
              if(channel==A){
               raw=readAdc0Ss2();
               final=((float)((raw*0.0008)));
               //final=(final-0.140)*2.175;                       // scaling it to 3.3V input to 0V.
               sprintf(str, "Voltage A:          %0.4f\n", final);
               putsUart0(str);
               }
               if(channel==B){
               raw=readAdc0Ss3();
               final=((float)((raw*0.0008)));
               //final=(final-0.140)*2.175;                       // scaling it to 3.3V input to 0V.
               sprintf(str, "Voltage B:          %0.4f\n", final);
               putsUart0(str);
               }
               waitMicrosecond(1000000);
            }}}}
}
