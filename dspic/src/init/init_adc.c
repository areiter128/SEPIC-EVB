/*
 * File:   init_adc.c
 * Author: M91406
 *
 * Created on July 9, 2019, 1:35 PM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "init_adc.h"

#define ADC_POWRUP_TIMEOUT  5000

volatile uint16_t init_adc_module(void) {
    
    // Make sure power to peripheral is enabled
    PMD1bits.ADC1MD = 0; // ADC Module Power Disable: ADC module power is enabled
    
    // ADCON1L: ADC CONTROL REGISTER 1 LOW
    ADCON1Lbits.ADON = 0; // ADC Enable: ADC module is off during configuration
    ADCON1Lbits.ADSIDL = 0; // ADC Stop in Idle Mode: Continues module operation in Idle mode
    
    // ADCON1H: ADC CONTROL REGISTER 1 HIGH
    ADCON1Hbits.SHRRES = 0b11; // Shared ADC Core Resolution Selection: 12-bit resolution ADC resolution = 12-bit (0...4095 ticks)
    ADCON1Hbits.FORM = 0; // Fractional Data Output Format: Integer

    // ADCON2L: ADC CONTROL REGISTER 2 LOW
    ADCON2Lbits.REFCIE = 0;; // Band Gap and Reference Voltage Ready Common Interrupt Enable: Common interrupt is disabled for the band gap ready event
    ADCON2Lbits.REFERCIE = 0; // Band Gap or Reference Voltage Error Common Interrupt Enable: Disabled
    ADCON2Lbits.EIEN = 1; // Early Interrupts Enable: The early interrupt feature is enabled
    ADCON2Lbits.PTGEN = 0; // External Conversion Request Interface: Disabled
    ADCON2Lbits.SHREISEL = 0b111; // Shared Core Early Interrupt Time Selection: Early interrupt is set and interrupt is generated 8 TADCORE clocks prior to when the data are ready
    ADCON2Lbits.SHRADCS = 0b0000001; // Shared ADC Core Input Clock Divider: 2:1 (minimum)

    // ADCON2H: ADC CONTROL REGISTER 2 HIGH
    ADCON2Hbits.SHRSAMC = 8; // Shared ADC Core Sample Time Selection: 8x TADs sampling time 
    ADCON2Hbits.REFERR = 0; // reset error flag
    ADCON2Hbits.REFRDY = 0; // reset bandgap status bit

    // ADCON3L: ADC CONTROL REGISTER 3 LOW
    ADCON3Lbits.REFSEL = 0b000; // ADC Reference Voltage Selection: AVDD-toAVSS
    ADCON3Lbits.SUSPEND = 0; // All ADC Core Triggers Disable: All ADC cores can be triggered
    ADCON3Lbits.SUSPCIE = 0; // Suspend All ADC Cores Common Interrupt Enable: Common interrupt is not generated for suspend ADC cores
    ADCON3Lbits.SUSPRDY = 0; // All ADC Cores Suspended Flag: ADC cores have previous conversions in progress
    ADCON3Lbits.SHRSAMP = 0; // Shared ADC Core Sampling Direct Control: use hardware trigger
    ADCON3Lbits.CNVRTCH = 0; // Software Individual Channel Conversion Trigger: Next individual channel conversion trigger can be generated (not used)
    ADCON3Lbits.SWLCTRG = 0; // Software Level-Sensitive Common Trigger: No software, level-sensitive common triggers are generated (not used)
    ADCON3Lbits.SWCTRG = 0; // Software Common Trigger: Ready to generate the next software common trigger (not used)
    ADCON3Lbits.CNVCHSEL = 0; // Channel Number Selection for Software Individual Channel Conversion Trigger: AN0 (not used)
    
    // ADCON3H: ADC CONTROL REGISTER 3 HIGH
    ADCON3Hbits.CLKSEL = 0b01; // ADC Module Clock Source Selection: AVCODIV
    ADCON3Hbits.CLKDIV = 0b000000; // ADC Module Clock Source Divider: 1 Source Clock Period
    ADCON3Hbits.SHREN = 0; // Shared ADC Core Enable: Shared ADC core is disabled
    ADCON3Hbits.C0EN = 0; // Dedicated ADC Core 0 Enable: Dedicated ADC Core 0 is disabled
    ADCON3Hbits.C1EN = 0; // Dedicated ADC Core 1 Enable: Dedicated ADC Core 1 is disabled
    
    // ADCON4L: ADC CONTROL REGISTER 4 LOW
    ADCON4Lbits.SAMC0EN = 0;  // Dedicated ADC Core 0 Conversion Delay Enable: Immediate conversion
    ADCON4Lbits.SAMC1EN = 0;  // Dedicated ADC Core 1 Conversion Delay Enable: Immediate conversion
    
    // ADCON4H: ADC CONTROL REGISTER 4 HIGH
    ADCON4Hbits.C0CHS = 0b00; // Dedicated ADC Core 0 Input Channel Selection: AN0
    ADCON4Hbits.C1CHS = 0b01; // Dedicated ADC Core 1 Input Channel Selection: ANA1

    // ADCON5L: ADC CONTROL REGISTER 5 LOW
    // ADCON5Lbits.SHRRDY: Shared ADC Core Ready Flag (read only)
    // ADCON5Lbits.C0RDY: Dedicated ADC Core 0 Ready Flag (read only)
    // ADCON5Lbits.C1RDY: Dedicated ADC Core 1 Ready Flag (read only)
    ADCON5Lbits.SHRPWR = 0; // Shared ADC Core Power Enable: ADC core is off
    ADCON5Lbits.C0PWR = 0; // Dedicated ADC Core 0 Power Enable: ADC core is off
    ADCON5Lbits.C1PWR = 0; // Dedicated ADC Core 1 Power Enable: ADC core is off
  
    // ADCON5H: ADC CONTROL REGISTER 5 HIGH
    ADCON5Hbits.WARMTIME = 0b1111; // ADC Dedicated Core x Power-up Delay: 32768 Source Clock Periods
    ADCON5Hbits.SHRCIE = 0; // Shared ADC Core Ready Common Interrupt Enable: Common interrupt is disabled for an ADC core ready event
    ADCON5Hbits.C0CIE = 0; // C1CIE: Dedicated ADC Core 0 Ready Common Interrupt Enable: Common interrupt is disabled
    ADCON5Hbits.C1CIE = 0; // C1CIE: Dedicated ADC Core 1 Ready Common Interrupt Enable: Common interrupt is disabled
    
    // ADCORExL: DEDICATED ADC CORE x CONTROL REGISTER LOW
    ADCORE1Lbits.SAMC = 0b0000000000;   // Dedicated ADC Core 1 Conversion Delay Selection: 2 TADCORE (minimum)
    ADCORE0Lbits.SAMC = 0b0000000000;   // Dedicated ADC Core 0 Conversion Delay Selection: 2 TADCORE (minimum)

    // ADCORExH: DEDICATED ADC CORE x CONTROL REGISTER HIGH
    ADCORE0Hbits.RES = 0b11; // ADC Core x Resolution Selection: 12 bit
    ADCORE0Hbits.ADCS = 0b0000000; // ADC Core x Input Clock Divider: 2 Source Clock Periods
    ADCORE0Hbits.EISEL = 0b111; // Early interrupt is set and an interrupt is generated 8 TADCORE clocks prior

    ADCORE1Hbits.RES = 0b11; // ADC Core x Resolution Selection: 12 bit
    ADCORE1Hbits.ADCS = 0b0000000; // ADC Core x Input Clock Divider: 2 Source Clock Periods
    ADCORE1Hbits.EISEL = 0b111; // Early interrupt is set and an interrupt is generated 8 TADCORE clocks prior
    
    return(1);
}

volatile uint16_t init_vin_adc(void) {

    // ANSELx: ANALOG SELECT FOR PORTx REGISTER
    ANSELCbits.ANSELC0 = 1; // Analog input is enabled and digital input is disabled for RC0 (Buck converter input voltage feedback)
    
    // ADLVLTRGL: ADC LEVEL-SENSITIVE TRIGGER CONTROL REGISTER LOW
    ADLVLTRGLbits.LVLEN12 = 0; // Input trigger is edge-sensitive

    // ADMOD0L: ADC INPUT MODE CONTROL REGISTER 0 LOW
    ADMOD0Hbits.DIFF13 = 0; // Differential-Mode for Corresponding Analog Inputs: Channel is single-ended
    ADMOD0Hbits.SIGN13 = 0; // Output Data Sign for Corresponding Analog Inputs: Channel output data are unsigned
    
    // ADEIEL: ADC EARLY INTERRUPT ENABLE REGISTER LOW
    ADEIELbits.EIEN12 = 1; // Early interrupt is enabled for the channel
    
    // ADIEL: ADC INTERRUPT ENABLE REGISTER LOW
    ADIELbits.IE12 = 1; // Common Interrupt Enable: Common and individual interrupts are disabled for the corresponding channel
    
    // ADTRIGnL/ADTRIGnH: ADC CHANNEL TRIGGER n(x) SELECTION REGISTERS LOW AND HIGH
    ADTRIG3Lbits.TRGSRC12 = 0b00101; // Trigger Source Selection for Corresponding Analog Inputs: PWM1 Trigger 2
    
    // ADCMPxCON: ADC DIGITAL COMPARATOR x CONTROL REGISTER
    ADCMP0CONbits.CHNL = 12; // Input Channel Number: 12=AN12
    ADCMP0CONbits.CMPEN = 1; // Comparator Enable: Comparator is enabled
    ADCMP0CONbits.IE = 0; // Comparator Common ADC Interrupt Enable: Common ADC interrupt will not be generated for the comparator
    ADCMP0CONbits.BTWN = 0; // Between Low/High Comparator Event: Disabled
    ADCMP0CONbits.HIHI = 1; // High/High Comparator Event: Enabled
    ADCMP0CONbits.HILO = 0; // High/Low Comparator Event: Disabled
    ADCMP0CONbits.LOHI = 0; // Low/High Comparator Event: Disabled
    ADCMP0CONbits.LOLO = 1; // Low/Low Comparator Event: Enabled
    
    // ADCMPxENL: ADC DIGITAL COMPARATOR x CHANNEL ENABLE REGISTER LOW
    ADCMP0ENLbits.CMPEN12 = 1; // Comparator Enable for Corresponding Input Channels: AN11 Enabled
    
    // ADCMPxLO: ADC COMPARARE REGISTER LOWER THRESHOLD VALUE REGISTER
    ADCMP0LO = 933; // R1=6.98kOhm, R2=1kOhm, G=0.751879699; 6Vin=933 ADC ticks

    // ADCMPxHI: ADC COMPARARE REGISTER UPPER THRESHOLD VALUE REGISTER
    ADCMP0HI = 2488; // R1=6.98kOhm, R2=1kOhm, G=0.751879699; 16Vin=2488 ADC ticks
    
    // ADFLxCON: ADC DIGITAL FILTER x CONTROL REGISTER
    ADFL0CONbits.FLEN = 0; // Filter Enable: Filter is disabled
    ADFL0CONbits.MODE = 0b11; // Filter Mode: Averaging mode (always 12-bit result 7 in oversampling mode 12-16bit wide)
    ADFL0CONbits.OVRSAM = 0b011; // Filter Averaging/Oversampling Ratio: 16x (result in the ADFLxDAT)
    ADFL0CONbits.IE = 0; // Filter Common ADC Interrupt Enable: Common ADC interrupt will not be generated for the filter
    ADFL0CONbits.FLCHSEL = 12; // Oversampling Filter Input Channel Selection: 12=AN12
    
    return(1);
}

volatile uint16_t init_buck_adc(void) {

    // ANSELx: ANALOG SELECT FOR PORTx REGISTER
    ANSELCbits.ANSELC1 = 1; // Analog input is enabled and digital input is disabled for RC1 (Buck converter output voltage feedback)
    
    // ADLVLTRGL: ADC LEVEL-SENSITIVE TRIGGER CONTROL REGISTER LOW
    ADLVLTRGLbits.LVLEN13 = 0; // Input trigger is edge-sensitive

    // ADMOD0L: ADC INPUT MODE CONTROL REGISTER 0 LOW
    ADMOD0Hbits.DIFF13 = 0; // Differential-Mode for Corresponding Analog Inputs: Channel is single-ended
    ADMOD0Hbits.SIGN13 = 0; // Output Data Sign for Corresponding Analog Inputs: Channel output data are unsigned
    
    // ADEIEL: ADC EARLY INTERRUPT ENABLE REGISTER LOW
    ADEIELbits.EIEN13 = 1; // Early interrupt is enabled for the channel
    
    // ADIEL: ADC INTERRUPT ENABLE REGISTER LOW
    ADIELbits.IE13 = 1; // Common Interrupt Enable: Common and individual interrupts are disabled for the corresponding channel
    
    // ADTRIGnL/ADTRIGnH: ADC CHANNEL TRIGGER n(x) SELECTION REGISTERS LOW AND HIGH
    ADTRIG3Lbits.TRGSRC13 = 0b01000; // Trigger Source Selection for Corresponding Analog Inputs: PWM3 Trigger 1
    
    // ADCMPxCON: ADC DIGITAL COMPARATOR x CONTROL REGISTER
    ADCMP1CONbits.CHNL = 13; // Input Channel Number: 13=AN13
    ADCMP1CONbits.CMPEN = 0; // Comparator Enable: Comparator is disabled
    ADCMP1CONbits.IE = 0; // Comparator Common ADC Interrupt Enable: Common ADC interrupt will not be generated for the comparator
    ADCMP1CONbits.BTWN = 0; // Between Low/High Comparator Event: Disabled
    ADCMP1CONbits.HIHI = 0; // High/High Comparator Event: Disabled
    ADCMP1CONbits.HILO = 0; // High/Low Comparator Event: Disabled
    ADCMP1CONbits.LOHI = 0; // Low/High Comparator Event: Disabled
    ADCMP1CONbits.LOLO = 0; // Low/Low Comparator Event: Disabled
   
    // ADCMPxENL: ADC DIGITAL COMPARATOR x CHANNEL ENABLE REGISTER LOW
    ADCMP1ENLbits.CMPEN13 = 0; // Comparator Enable for Corresponding Input Channels: AN13 Disabled
    
    // ADCMPxLO: ADC COMPARARE REGISTER LOWER THRESHOLD VALUE REGISTER
    ADCMP1LO = 620; // R1=1kOhm, R2=1kOhm, G=0.5000; 1Vin=620 ADC ticks

    // ADCMPxHI: ADC COMPARARE REGISTER UPPER THRESHOLD VALUE REGISTER
    ADCMP1HI = 2358; // R1=1kOhm, R2=1kOhm, G=0.5000; 3.8Vin=2358 ADC ticks
    
    // ADFLxCON: ADC DIGITAL FILTER x CONTROL REGISTER
    ADFL1CONbits.FLEN = 0; // Filter Enable: Filter is disabled
    ADFL1CONbits.MODE = 0b11; // Filter Mode: Averaging mode (always 12-bit result 7 in oversampling mode 12-16bit wide)
    ADFL1CONbits.OVRSAM = 0b001; // Filter Averaging/Oversampling Ratio: 16x (result in the ADFLxDAT)
    ADFL1CONbits.IE = 0; // Filter Common ADC Interrupt Enable: Common ADC interrupt will not be generated for the filter
    ADFL1CONbits.FLCHSEL = 13; // Oversampling Filter Input Channel Selection: 13=AN13

    return(1);
}

volatile uint16_t init_boost_adc(void) {
    
    // ANSELx: ANALOG SELECT FOR PORTx REGISTER
    ANSELDbits.ANSELD10 = 1; // Analog input is enabled and digital input is disabled for RD10 (Boost converter output voltage feedback)
    
    // ADLVLTRGL: ADC LEVEL-SENSITIVE TRIGGER CONTROL REGISTER LOW
    ADLVLTRGHbits.LVLEN18 = 0; // Input trigger is edge-sensitive
    
    // ADMOD0L: ADC INPUT MODE CONTROL REGISTER 0 LOW
    ADMOD1Lbits.DIFF18 = 0; // Differential-Mode for Corresponding Analog Inputs: Channel is single-ended
    ADMOD1Lbits.SIGN18 = 0; // Output Data Sign for Corresponding Analog Inputs: Channel output data are unsigned
    
    // ADEIEL: ADC EARLY INTERRUPT ENABLE REGISTER LOW
    ADEIEHbits.EIEN18 = 1; // Early interrupt is enabled for the channel
    
    // ADIEL: ADC INTERRUPT ENABLE REGISTER LOW
    ADIEHbits.IE18 = 1; // Common Interrupt Enable: Common and individual interrupts are disabled for the corresponding channel

    // ADTRIGnL/ADTRIGnH: ADC CHANNEL TRIGGER n(x) SELECTION REGISTERS LOW AND HIGH
    ADTRIG4Hbits.TRGSRC18 = 0b01010; // Trigger Source Selection for Corresponding Analog Inputs: PWM4 Trigger 1
    
    // ADCMPxCON: ADC DIGITAL COMPARATOR x CONTROL REGISTER
    ADCMP2CONbits.CHNL = 18; // Input Channel Number: 13=AN13
    ADCMP2CONbits.CMPEN = 0; // Comparator Enable: Comparator is disabled
    ADCMP2CONbits.IE = 0; // Comparator Common ADC Interrupt Enable: Common ADC interrupt will not be generated for the comparator
    ADCMP2CONbits.BTWN = 0; // Between Low/High Comparator Event: Disabled
    ADCMP2CONbits.HIHI = 0; // High/High Comparator Event: Disabled
    ADCMP2CONbits.HILO = 0; // High/Low Comparator Event: Disabled
    ADCMP2CONbits.LOHI = 0; // Low/High Comparator Event: Disabled
    ADCMP2CONbits.LOLO = 0; // Low/Low Comparator Event: Disabled
    
    // ADCMPxENL: ADC DIGITAL COMPARATOR x CHANNEL ENABLE REGISTER LOW
    ADCMP2ENHbits.CMPEN18 = 0; // Comparator Enable for Corresponding Input Channels: AN18 Disabled
    
    // ADCMPxLO: ADC COMPARARE REGISTER LOWER THRESHOLD VALUE REGISTER
    ADCMP2LO = 1399; // R1=6.98kOhm, R2=1kOhm, G=0.751879699; 9Vin=1399 ADC ticks

    // ADCMPxHI: ADC COMPARARE REGISTER UPPER THRESHOLD VALUE REGISTER
    ADCMP2HI = 2799; // R1=6.98kOhm, R2=1kOhm, G=0.751879699; 18Vin=2799 ADC ticks
    
    // ADFLxCON: ADC DIGITAL FILTER x CONTROL REGISTER
    ADFL2CONbits.FLEN = 0; // Filter Enable: Filter is disabled
    ADFL2CONbits.MODE = 0b11; // Filter Mode: Averaging mode (always 12-bit result 7 in oversampling mode 12-16bit wide)
    ADFL2CONbits.OVRSAM = 0b001; // Filter Averaging/Oversampling Ratio: 16x (result in the ADFLxDAT)
    ADFL2CONbits.IE = 0; // Filter Common ADC Interrupt Enable: Common ADC interrupt will not be generated for the filter
    ADFL2CONbits.FLCHSEL = 18; // Oversampling Filter Input Channel Selection: 18=AN18
    
    return(1);
}

volatile uint16_t launch_adc(void) {

    volatile uint16_t timeout=0;
    
    // If ADC Module is already turned on, ADC is running => skip launch_adc())
    if(ADCON1Lbits.ADON) return(1); 
    
    ADCON1Lbits.ADON = 1; // ADC Enable: ADC module is enabled first

    ADCON5Lbits.SHRPWR = 1; // Enabling Shared ADC Core analog circuits power
    while((!ADCON5Lbits.SHRRDY) && (timeout++<ADC_POWRUP_TIMEOUT));
    if((!ADCON5Lbits.SHRRDY) || (timeout>=ADC_POWRUP_TIMEOUT)) return(0);
    ADCON3Hbits.SHREN  = 1; // Enable Shared ADC digital circuitry
        
    ADCON5Lbits.C0PWR = 0; // Dedicated ADC Core 0 Power Enable: ADC core is off
//    while((!ADCON5Lbits.C0RDY) && (timeout++<ADC_POWRUP_TIMEOUT));
//    if((!ADCON5Lbits.C0RDY) || (timeout>=ADC_POWRUP_TIMEOUT)) return(0);
    ADCON3Hbits.C0EN  = 0; // Dedicated Core 0 is not enabled

    ADCON5Lbits.C1PWR = 0; // Dedicated ADC Core 1 Power Enable: ADC core is off
//    while((!ADCON5Lbits.C1RDY) && (timeout++<ADC_POWRUP_TIMEOUT));
//    if((!ADCON5Lbits.C1RDY) || (timeout>=ADC_POWRUP_TIMEOUT)) return(0);
    ADCON3Hbits.C1EN  = 0; // Dedicated Core 1 is not enabled

    // INITIALIZE AN12 INTERRUPTS (Board Input Voltage)
    IPC25bits.ADCAN12IP = 0;   // Interrupt Priority Level 0
    IFS6bits.ADCAN12IF = 0;    // Reset Interrupt Flag Bit
    IEC6bits.ADCAN12IE = 0;    // Disable ADCAN12 Interrupt 

     // INITIALIZE AN13 INTERRUPTS (Buck Output Voltage)
    IPC26bits.ADCAN13IP = 5;   // Interrupt Priority Level 5
    IFS6bits.ADCAN13IF = 0;    // Reset Interrupt Flag Bit
    IEC6bits.ADCAN13IE = 1;    // Enable ADCAN13 Interrupt 
    
    // INITIALIZE AN18 INTERRUPTS (Boost Output Voltage)
    IPC27bits.ADCAN18IP = 5;   // Interrupt Priority Level 5
    IFS6bits.ADCAN18IF = 0;    // Reset Interrupt Flag Bit
    IEC6bits.ADCAN18IE = 1;    // Enable ADCAN18 Interrupt 
    
    return(1);
}


