#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "tm4c123gh6pm.h"

#define MAX_CHARS 80
#define MAX_FIELDS 6
#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")
#define pi 3.14159265359

//Global Variables
char str[MAX_CHARS+1];
uint8_t pos[MAX_FIELDS];
uint8_t argCount=0;
uint8_t count = 0;
uint8_t count2=MAX_CHARS;
char str1[MAX_CHARS+1];
char str2[MAX_CHARS+1];
char str3[MAX_CHARS+1];
char str4[MAX_CHARS+1];
char str5[MAX_CHARS+1];
char set[6];
uint32_t delta_phi1=0;
uint32_t delta_phi2=0;
uint32_t phi1=0;
uint32_t phi2=0;
uint8_t ch1 = 0;
uint8_t ch2 = 1;
float freq1=0;
float freq2=0;
float freq=0;
uint16_t LUT[3][4096];
uint8_t nCycle = 0;
uint8_t run = 0;
uint8_t n2Cycle = 0;
uint32_t cycleCount = 0;
float voltOut=0;

// PortE masks
#define AIN0_MASK 8
#define AIN1_MASK 4

// PortD masks
#define TX_MASK 8
#define LDAC_MASK 4
#define FSS_MASK 2
#define CLK_MASK 1

// PortC masks
#define FREQ_IN_MASK 64

// Pin bitbands
#define LDAC           (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4)))


// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A peripherals
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;


    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= 2;                           // enable output on UART0 TX pin: default, added for clarity
    GPIO_PORTA_DEN_R |= 3;                           // enable digital on UART0 pins: default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // use peripheral to drive PA0, PA1: default, added for clarity
    GPIO_PORTA_PCTL_R &= 0xFFFFFF00;                 // set fields for PA0 and PA1 to zero
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                     // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other UARTs in same status
    delay4Cycles();                                  // wait 4 clock cycles
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;

    // Enable clocks
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0 | SYSCTL_RCGCADC_R1;
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE;

    // Configure AIN0 and AI1 as an analog input
       GPIO_PORTE_AFSEL_R |= AIN0_MASK | AIN1_MASK;                 // select alternative functions for AIN0 and AIN1 on Port E (PE3 and PE2)
       GPIO_PORTE_DEN_R &= ~AIN0_MASK & ~AIN1_MASK;                  // turn off digital operation on pin PE3 and PE2
       GPIO_PORTE_AMSEL_R |= AIN0_MASK | AIN1_MASK;                 // turn on analog operation on pin PE3 and PE2

       // Configure ADC
       ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
       ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
       ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
       ADC0_SSMUX3_R = 0;                               // set first sample to AIN0
       ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
       ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

       ADC1_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
       ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
       ADC1_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
       ADC1_SSMUX3_R = 1;                               // set first sample to AIN1
       ADC1_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
       ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

    // Enable clocks
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R1;
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD;

    // Configure LDAC on PD2
    GPIO_PORTD_DIR_R |= LDAC_MASK;                       // make bit 2 an output
    GPIO_PORTD_DR2R_R |= LDAC_MASK;                      // set drive strength to 2mA
    GPIO_PORTD_DEN_R |= LDAC_MASK              ;         // enable bit 1 for digital

    // Configure SSI1 pins for SPI configuration
    GPIO_PORTD_DIR_R |= TX_MASK | FSS_MASK | CLK_MASK; // make SSI1 TX, FSS, and CLK outputs
    GPIO_PORTD_DR2R_R |= TX_MASK | FSS_MASK | CLK_MASK; // set drive strength to 2mA
    GPIO_PORTD_AFSEL_R |= TX_MASK | FSS_MASK | CLK_MASK; // select alternative functions
    GPIO_PORTD_PCTL_R = GPIO_PCTL_PD3_SSI1TX | GPIO_PCTL_PD1_SSI1FSS | GPIO_PCTL_PD0_SSI1CLK; // map alt fns to SSI1
    GPIO_PORTD_DEN_R |= TX_MASK | FSS_MASK | CLK_MASK; // enable digital operation
    GPIO_PORTD_PUR_R |= CLK_MASK;                      // SCLK must be enabled when SPO=1 (see 15.4)

    // Configure the SSI1 as a SPI master, mode 3, 16bit operation, 1 MHz bit rate
    SSI1_CR1_R &= ~SSI_CR1_SSE;                        // turn off SSI1 to allow re-configuration
    SSI1_CR1_R = 0;                                    // select master mode
    SSI1_CC_R = 0;                                     // select system clock as the clock source
    SSI1_CPSR_R = (40/4);                                  // set bit rate to 4 MHz (if SR=0 in CR0)
    SSI1_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 16-bit
    SSI1_CR1_R |= SSI_CR1_SSE;                         // turn on SSI1

    // Enable clocks
      SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;


      // Configure Timer 1 as the time base
      TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
      TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
      TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
      TIMER1_TAILR_R = 400;                       // set load value to 40e6 for 1 Hz interrupt rate
      TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
      NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
      TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

}




// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}


// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}


// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char*str)
{
    uint8_t i;
    for (i = 0; i <strlen(str) ; i++)
      putcUart0(str[i]);
}

void printline(){putsUart0("\r\n----------------------------------------------------------------\r\n");}



void timer1Isr()
{
    if(run)
    {

    if (nCycle>0 & cycleCount>0)
     {
        cycleCount--;
    LDAC = 0;                         // Pull LDAC Low
    LDAC = 1;                         // Pull LDAC High
    phi1 += delta_phi1;                //Increment phi1 and 2
    phi2 += delta_phi2;
    SSI1_DR_R = LUT[ch1][phi1>>20];               // write new data to SSI1
    SSI1_DR_R = LUT[ch2][phi2>>20];               // write new data to SSI1
    if(cycleCount==0){delta_phi1=0; delta_phi2= 0; run = 0;}
    }
    else if( nCycle==0 )
         {
        LDAC = 0;                         // Pull LDAC Low
        LDAC = 1;                         // Pull LDAC High
        phi1 += delta_phi1;                //Increment phi1 and 2
        phi2 += delta_phi2;
        SSI1_DR_R = LUT[ch1][phi1>>20];
        SSI1_DR_R = LUT[ch2][phi2>>20];               // write new data to SSI1

        }
    }
    else
    {
        LDAC = 0;                         // Pull LDAC Low
        LDAC = 1;                         // Pull LDAC High
        SSI1_DR_R = (1 << 12) | (1 << 13) | (1 << 11)  -51;               // write new data to SSI1
        SSI1_DR_R = (1 << 12) | (1 << 13) | (1 << 14)| (1 << 11) - 39;    // write new data to SSI1
    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}

void getString(char str[], uint8_t maxChars)
{

    char c; count = 0;
    Loop1:
    c = getcUart0();
      if(c == 8 | c == 127)
        {
            if(count>0){count--;goto Loop1;}
            else{goto Loop1;}
        }

      if(c == 10 | c == 13){str[count] = 0x00; return;}
      else
        {
            if(c>=32){str[count++] = c;}
            else goto Loop1;
        }

        if(count == maxChars)
        {
            str[count] = 0x00;
            putsUart0("You have exceeded the maximum characters, you typed\r\n");
            return;
         }
        else goto Loop1;
}

void posArg(char*str)
{
    uint8_t i; argCount = 0;
    for (i = 0; i < count; i++)
    {
        if(str[i]==32 | str[i]==44)
        {
            str[i]=0x00;
        }
        if(str[i]!=0x00 & str[i-1]==0x00)
        {
            pos[argCount] = i;
            argCount++;
        }

    }
    return;
}


void parseString(char* str,  uint8_t* pos, uint8_t argCount)
{
    uint8_t j; uint8_t i; uint8_t temp_count; char temp_string[81];
    printline(); putsUart0("Processing the input string as: ");
    for(j=0;j<argCount;j++)
    {
        temp_count = 0;
        for (i = 0; i < 80; i++)
        {
            temp_string[i] = 0x00;
        }

        putcUart0(0x5B);
        for(i=pos[j];i<count;i++)
            {
            if(str[i]==0x00){break;}
            else
            {
                putcUart0(str[i]);
                temp_string[temp_count++] = str[i];
            }
            }
        if(j==0){strcpy(str1,temp_string);}
        else if(j==1){strcpy(str2,temp_string);}
        else if(j==2){strcpy(str3,temp_string);}
        else if(j==3){strcpy(str4,temp_string);}
        else if(j==4){strcpy(str5,temp_string);}
        putcUart0(0x5D); putcUart0(0x20);
    }
    return;
}

// Request and read one sample from SS3
int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO

}

// Request and read one sample from SS3
int16_t readAdc1Ss3()
{
    ADC1_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC1_SSFIFO3_R;                           // get single result from the FIFO

}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1x10^6
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6x10^6
    __asm("             CBZ  R1, WMS_DONE1");   // (5+1*3)x10^6
    __asm("             NOP");                  // 5x10^6
    __asm("             NOP");                  // 5x10^6
    __asm("             B    WMS_LOOP1");       // (5*2)x10^6 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1x10^6
    __asm("             CBZ  R0, WMS_DONE0");   // 1x(10^6-1)+3
    __asm("             NOP");                  // 1x10^6-1+0
    __asm("             B    WMS_LOOP0");       //(1*2)x(10^6-1)+0
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error 40x10^6-1
}
int sgn(float value)
{
    if(value>0){return 1;}
    else return -1;
}

int mod(int16_t value)
{
    if(value>0){return value;}
        else return -value;
}

void runCmd()
{
    putsUart0("\r\nType Run to start the signal\n\r");
    getString(str,MAX_CHARS);     putcUart0(0x0a);
    putcUart0(0x0d);
    putsUart0("\r\nYou Typed:"); putsUart0(str);    putcUart0(0x0a);
    putcUart0(0x0d);
    if(strcmp("run", str)==0)
    {
        cycleCount = (uint32_t)(n2Cycle*(1/freq)*100000);
        nCycle = n2Cycle;
        putsUart0("\r\nOutputting Waveform Now\n\r");
        run =1;
        return;

    }
    else{
        putsUart0("\r\nSorry Invalid Command\n\r");
    }

}


void lookUp()
{
    uint16_t i;
    uint8_t channel = atoi(str2); float off = atof(str3); float amp = atof(str4);
    if(strcmp("gain", str1)==0){float off = 0; float amp = 4; }
    else{freq = atof(str5);}

    if(channel==1)
    {
        freq1  = freq;
        if(ch1|ch2==1){ch1 = 2;}
        else if(ch1|ch2==3){ch1 = 0;}
        else{ch1 = 1;}
        if(strcmp("square", str1)==0)
        {
            for(i=0; i<4096; i++){ LUT[ch1][i] = (1 << 12) | (1 << 13) | (uint16_t)(409.6*(-0.94)*(off + amp*sgn(sin(2*pi*i/4096))) -51+2048);}
        }
    else if(strcmp("sine", str1)==0 | strcmp("gain", str1)==0){
        for(i=0; i<4096; i++){
        LUT[ch1][i] = (1 << 12) | (1 << 13) | (uint16_t)(409.6*(-0.94)*(off + amp*sin(2*pi*i/4096)) -51+2048);}
    }
    else if(strcmp("dc", str1)==0){
            for(i=0; i<4096; i++){
            LUT[ch1][i] = (1 << 12) | (1 << 13) | (uint16_t)(409.6*(-0.94)*(off) -51+2048);}
        }
    else if(strcmp("diff", str1)==0){ off = voltOut;
                for(i=0; i<4096; i++){
                LUT[ch1][i] = (1 << 12) | (1 << 13) | (uint16_t)(409.6*(-0.94)*(off) -51+2048);}
            }
    else if(strcmp("duty", str1)==0){ off = 4; freq1 = amp;
                for(i=0; i<4096; i++)
                {
                    if(i<atof(str3)*40.96){LUT[ch1][i] = (1 << 12) | (1 << 13) | (uint16_t)(409.6*(-0.94)*(off) -51+2048);}
                    else{LUT[ch1][i] = (1 << 12) | (1 << 13) | (uint16_t)(409.6*(-0.94)*(0) -51+2048);}
            }
    }
    else if(strcmp("tri", str1)==0){
                for(i=0; i<4096; i++){
                    uint16_t value = 2048-i;
                LUT[ch1][i] = (1 << 12) | (1 << 13) | (uint16_t)(409.6*(-0.94)*(off +amp*mod(value)/2048) -51+2048);}
            }
    else if(strcmp("sawt", str1)==0){
                for(i=0; i<4095; i++){
                LUT[ch1][i] = (1 << 12) | (1 << 13) | (uint16_t)(409.6*(-0.94)*(off + (amp)*i/4095) -51+2048);}
                LUT[ch1][4095] = (1 << 12) | (1 << 13) | (uint16_t)(409.6*(-0.94)*(off) + -51+2048);
            }
    }
    else if(channel==2)
    {
        freq2  = freq;
        if(ch1|ch2==1){ch2 = 2;}
        else if(ch1|ch2==3){ch2 = 0;}
        else {ch2 = 1;}
        if(strcmp("square", str1)==0)
               {
            for(i=0; i<4096; i++){ LUT[ch2][i] = (1 << 12) | (1 << 13) | (1 << 15) | (uint16_t)(409.6*(-0.93)*(off + amp*sgn(sin(2*pi*i/4096))) -39+2048);}

               }
        else if(strcmp("diff", str1)==0){ off = voltOut;
                    for(i=0; i<4096; i++){
                    LUT[ch1][i] = (1 << 12) | (1 << 13) | (1 << 15) | (uint16_t)(409.6*(-0.94)*(off) -51+2048);}
                }
        else if(strcmp("sine", str1)==0 | strcmp("gain", str1)==0)
            {
            for(i=0; i<4096; i++){
        LUT[ch2][i] = (1 << 12) | (1 << 13) | (1 << 15) | (uint16_t)(409.6*(-0.93)*(off + amp*sin(2*pi*i/4096)) -39+2048);}
            }
        else if(strcmp("duty", str1)==0){ off = 4; freq2 = amp;
                    for(i=0; i<4096; i++)
                    {
                        if(i<atof(str3)*40.96){LUT[ch1][i] = (1 << 12) | (1 << 13) | (1 << 15) | (uint16_t)(409.6*(-0.94)*(off) -51+2048);}
                        else{LUT[ch1][i] = (1 << 12) | (1 << 13) | (1 << 15) | (uint16_t)(409.6*(-0.94)*(0) -51+2048);}
                }
        }
        else if(strcmp("dc", str1)==0)
                    {
                    for(i=0; i<4096; i++){
                LUT[ch2][i] = (1 << 12) | (1 << 13) | (1 << 15) | (uint16_t)(409.6*(-0.93)*(off) -39+2048);}
                    }
        else if(strcmp("tri", str1)==0)
                            {
                            for(i=0; i<4096; i++){
                                uint16_t value = 2048-i;
                        LUT[ch2][i] = (1 << 12) | (1 << 13) | (1 << 15) | (uint16_t)(409.6*(-0.93)*(off +amp*mod(value)/2048) -39+2048);}
                            }
        else if(strcmp("sawt", str1)==0)
                            {
                            for(i=0; i<4095; i++){
                        LUT[ch2][i] = (1 << 12) | (1 << 13) | (1 << 15) | (uint16_t)(409.6*(-0.93)*(off + amp*i/4096) -39+2048);}
                        LUT[ch2][4095] = (1 << 12) | (1 << 13) | (1 << 15) | (uint16_t)(409.6*(-0.93)*(off)  -39+2048);
                            }
    }
  delta_phi1 = ((1<<20)/100000)*4096*freq1;
  delta_phi2 = ((1<<20)/100000)*4096*freq2;

  if(strcmp("gain", str1)!=0){runCmd();}
  if(strcmp("gain", str1)==0){run=1;}
}





void isCommand()
{
    // Convert the parameters in float/int data type
    uint8_t channel = atoi(str2); float off = atof(str3); float amp = atof(str4);
    if(strcmp("sine", str1)==0)
    {
        putsUart0("\r\nSine command received");
        if(strcmp("1", str2)!=0&strcmp("2", str2)!=0){putsUart0("\r\nInvalid channel no.");}
        else if(argCount<4){putsUart0("\r\nInsufficient Arguments");}
        else{
        putsUart0("\r\nFollowing output will be generated at channel no.");
        putsUart0(str2);    printline();
        putsUart0("Waveform              :");  putsUart0(str1);
        putsUart0("\r\nOffset Value          :");  putsUart0(str3);
        putsUart0("\r\nAmplitude             :");  putsUart0(str4);
        putsUart0("\r\nFrequency             :");  putsUart0(str5);
        printline();
        lookUp();}
    }
    else if(strcmp("diff", str1)==0)
            {
            uint16_t RawIn,VoltInInt,VoltInFrac; float VoltIn;
            RawIn = readAdc0Ss3();
            VoltIn = (float)(RawIn)*3.3/4096;
            VoltInInt = (uint16_t)VoltIn;
            VoltInFrac = (VoltIn - (uint16_t)VoltIn)*1000;
            sprintf(str,"Voltage Input is %4u.%.3u",VoltInInt, VoltInFrac);
            putsUart0(str);
            sprintf(str,"Voltage Output will be -%4u.%.3u",VoltInInt, VoltInFrac);
            putsUart0(str);
            voltOut = -VoltIn;
            lookUp();
            }
    else if(strcmp("voltage", str1)==0)
        {
            putsUart0("\r\nVoltage command received, collecting samples\n\r");

                   // Initialize an array x consisting of 16 elements
                   uint16_t x1[100]; uint16_t i;
                   for(i = 0; i < 100; i++){x1[i] = 0;}
                   uint16_t index1 = 0; uint32_t sum1 = 0; float Vin1 = 0;
                   uint16_t raw1=0;
                   uint8_t channel = atoi(str2);
                   char str10[100];

                   while(true)
                      {
                       if(channel==1){raw1 = readAdc0Ss3();}
                       else if(channel==2){raw1 = readAdc1Ss3();}
                       else{putsUart0("Invalid Channel no."); return;}

                           // FIR sliding average filter with circular addressing

                           sum1 += raw1; //new sensor reading
                           sum1 -= x1[index1];
                           x1[index1] = raw1;
                           index1 = (index1 + 1);
                           Vin1 = (3.3 * (sum1/100) )/4096;

                           //putsUart0(".");
                           if(index1==99)
                           {
                           // display raw ADC value

                           sprintf(str10, "Channel no.        %7lu", channel);
                           putsUart0(str10);
                          printline();
                           sprintf(str10, "12-bit Value (4096):        %7lu", sum1 / 100);
                           putsUart0(str10); putcUart0(0x0a); putcUart0(0x0d);
                           sprintf(str10, "Voltage :           %7lu Volts and %7lu milliVolts",(uint16_t)Vin1,(uint16_t)( (Vin1 - (uint16_t)Vin1) *1000));
                           putsUart0(str10);
                           printline();
                           break;
                          }


                       }

        }
    else if(strcmp("stop", str1)==0)
        {
         putsUart0("Stopping the waveform");
         run = 0;
        }


    else if(strcmp("dc", str1)==0)
        {
            putsUart0("\r\nDC command received, outputting a voltage of  "); putsUart0(str3);
            putsUart0("  Volts on channel no. "); putsUart0(str2); putsUart0("\n\r");
            lookUp();
        }
    else if(strcmp("square", str1)==0)
            {
        putsUart0("\r\nSquare command received");
        if(strcmp("1", str2)!=0&strcmp("2", str2)!=0){putsUart0("\r\nInvalid channel no.");}
                else if(argCount<4){putsUart0("\r\nInsufficient Arguments");}
                else{
                putsUart0("\r\nFollowing output will be generated at channel no.");
                putsUart0(str2);    printline();
                putsUart0("Waveform              :");  putsUart0(str1);
                putsUart0("\r\nOffset Value          :");  putsUart0(str3);
                putsUart0("\r\nAmplitude             :");  putsUart0(str4);
                putsUart0("\r\nFrequency             :");  putsUart0(str5);
                printline();
                lookUp();}
            }
    else if(strcmp("reset", str1)==0 )
        {
            putsUart0("\r\nReset command received, resetting........");
            NVIC_APINT_R = NVIC_APINT_VECTKEY| NVIC_APINT_SYSRESETREQ;
            lookUp();
        }
    else if(strcmp("gain", str1)==0 )
            {
                putsUart0("\r\nGain Command recieved with following parameters:"); printline();
                putsUart0("\r\nChannel no.   :"); putsUart0(str2);
                putsUart0("\r\nFrequency 1   :"); putsUart0(str3);
                putsUart0("\r\nFrequency 2   :"); putsUart0(str4); printline();
                uint16_t i=atoi(str3); float gain=0; uint16_t localIndex=0; char str11[100]; float raw1=0; float raw2=0;

                while(true)
                {
                    if(i>atoi(str4)){i=atoi(str4);}
                    freq = i;
                    lookUp();
                    waitMicrosecond(1000000);
                    raw1 = (float)(readAdc0Ss3());
                    raw2 = (float)(readAdc1Ss3());
                    gain = raw1/raw2;
                    *str11=0;
                    //gain = 1.1112;
                    sprintf(str11, "%lu. ", localIndex+1);
                    putsUart0(str11); *str11=0;
                    float fracGain = (gain - (uint16_t)gain)*1000;
                    uint16_t intGain = (uint16_t)fracGain;
                    sprintf(str11, "Gain = %4u.%.3u", (uint16_t)gain, intGain   );
                    putsUart0(str11); *str11=0;
                    sprintf(str11, "\n\rFrequency = %4lu", i);
                    putsUart0(str11);
                    printline();

                    if(i==atoi(str4)){break;}

                    i = 2*i;
                    localIndex++;
                }
                run=0;
            }
    else if(strcmp("duty", str1)==0 )
    {
        putsUart0("\r\nDuty Cycle command received");
        putsUart0("\r\nFollowing output will be generated at channel no.");
        putsUart0(str2);    printline();
        putsUart0("\r\nDuty Cycle          :");  putsUart0(str3);
        lookUp();
    }
    else if(strcmp("tri", str1)==0)
            {
        putsUart0("\r\nTriangle command received");
               if(strcmp("1", str2)!=0&strcmp("2", str2)!=0){putsUart0("\r\nInvalid channel no.");}
                       else if(argCount<4){putsUart0("\r\nInsufficient Arguments");}
                       else{
                       putsUart0("\r\nFollowing output will be generated at channel no.");
                       putsUart0(str2);    printline();
                       putsUart0("Waveform              :");  putsUart0(str1);
                       putsUart0("\r\nOffset Value          :");  putsUart0(str3);
                       putsUart0("\r\nAmplitude             :");  putsUart0(str4);
                       putsUart0("\r\nFrequency             :");  putsUart0(str5);
                       printline();
                       lookUp();}

            }
    else if(strcmp("sawt", str1)==0)
                {
        putsUart0("\r\nSaw Tooth command received");
               if(strcmp("1", str2)!=0&strcmp("2", str2)!=0){putsUart0("\r\nInvalid channel no.");}
                       else if(argCount<4){putsUart0("\r\nInsufficient Arguments");}
                       else{
                       putsUart0("\r\nFollowing output will be generated at channel no.");
                       putsUart0(str2);    printline();
                       putsUart0("Waveform              :");  putsUart0(str1);
                       putsUart0("\r\nOffset Value          :");  putsUart0(str3);
                       putsUart0("\r\nAmplitude             :");  putsUart0(str4);
                       putsUart0("\r\nFrequency             :");  putsUart0(str5);
                       printline();
                       lookUp();
                }

                }

    else
    {

        putsUart0("\r\nSorry that is an invalid Command");
    }

    return;
}

void nCycleIn(char *str1, char* str2)
{
    if(strcmp("cont", str2)==0)
            {
        nCycle = 0;
        return;
            }
            else
                {
                n2Cycle = atoi(str2);
                }
}


int main(void)
{


    // Initialize hardware
    initHw();

    *str=0;

    putsUart0("Please enter nCycle command\r\n");
    getString(str,MAX_CHARS);
    posArg(str); // To process the string, calculate the no. of arguments and their positions
    parseString(str,pos,argCount); printline();

    nCycleIn(str1,str2);

    while(1)
    {

        waitMicrosecond(1000000);
    ///////////////////////////////////////////////////
                     //Step2//
    //////////////////////////////////////////////////
    *str = 0; *str1=0; *str2=0; *str3=0; *str4=0; *str5=0;

    putcUart0(0x0a);
    putcUart0(0x0d);
    printline();
    putsUart0("Please enter the command, followed by enter key\r\n");
    getString(str,MAX_CHARS);
    putsUart0("User Input: "); putsUart0(str);  printline();
    run = 0;


    ///////////////////////////////////////////////////
                     //Step3//
    //////////////////////////////////////////////////

    posArg(str); // To process the string, calculate the no. of arguments and their positions
    parseString(str,pos,argCount); printline();
    freq = atof(str5);

    ///////////////////////////////////////////////////////
                    //Step all//
    ////////////////////////////////////////////////////////

   isCommand();

    }

}
