/*
 * File:   sepic_hwdescr.c
 * Author: M91406
 *
 * Created on July 25, 2019, 11:13 PM
 */


#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
#include <stdbool.h>

#include "sepic_hwdescr.h"


// Digital-To-Analog Converter and High Speed Comparator Special Function Register Set 

typedef struct {
    volatile uint16_t ANSELx;
    volatile uint16_t TRISx;
    volatile uint16_t PORTx;
    volatile uint16_t LATx;
    volatile uint16_t ODCx;
    volatile uint16_t CNPUx;
    volatile uint16_t CNPDx;
    volatile uint16_t CNCONx;
    volatile uint16_t CNEN0x;
    volatile uint16_t CNSTATx;
    volatile uint16_t CNEN1x;
    volatile uint16_t CNFx;
} P33C_GPIO_INSTANCE_t;
#define GPIO_SFR_OFFSET  ((volatile uint16_t)&ANSELB - (volatile uint16_t)&ANSELA)

typedef struct {
    volatile uint16_t DACxCONL;
    volatile uint16_t DACxCONH;
    volatile uint16_t DACxDATL;
    volatile uint16_t DACxDATH;
    volatile uint16_t SLPxCONL;
    volatile uint16_t SLPxCONH;
    volatile uint16_t SLPxDAT;
} P33C_DAC_INSTANCE_t;
#define DAC_SFR_OFFSET  ((volatile uint16_t)&DAC2CONL - (volatile uint16_t)&DAC1CONL)

// PWM Generator Special Function Register Set 

typedef struct {
    volatile uint16_t PGxCONL;
    volatile uint16_t PGxCONH;
    volatile uint16_t PGxSTAT;
    volatile uint16_t PGxIOCONL;
    volatile uint16_t PGxIOCONH;
    volatile uint16_t PGxEVTL;
    volatile uint16_t PGxEVTH;
    volatile uint16_t PGxFPCIL;
    volatile uint16_t PGxFPCIH;
    volatile uint16_t PGxCLPCIL;
    volatile uint16_t PGxCLPCIH;
    volatile uint16_t PGxFFPCIL;
    volatile uint16_t PGxFFPCIH;
    volatile uint16_t PGxSPCIL;
    volatile uint16_t PGxSPCIH;
    volatile uint16_t PGxLEBL;
    volatile uint16_t PGxLEBH;
    volatile uint16_t PGxPHASE;
    volatile uint16_t PGxDC;
    volatile uint16_t PGxDCA;
    volatile uint16_t PGxPER;
    volatile uint16_t PGxTRIGA;
    volatile uint16_t PGxTRIGB;
    volatile uint16_t PGxTRIGC;
    volatile uint16_t PGxDTL;
    volatile uint16_t PGxDTH;
    volatile uint16_t PGxCAP;
} P33C_PWM_INSTANCE_t;
#define PWM_SFR_OFFSET  ((volatile uint16_t)&PG2CONL - (volatile uint16_t)&PG1CONL)

volatile uint16_t init_sepic_pcmc(volatile uint16_t pwm_Instance, volatile uint16_t dac_Instance, volatile uint16_t gpio_Instance) {

    volatile P33C_GPIO_INSTANCE_t* gpio;
    volatile P33C_DAC_INSTANCE_t* dac;
    volatile P33C_PWM_INSTANCE_t* pg;
    volatile uint16_t dummy=0;

    // CAPTURE MEMORY ADDRESS OF GIVEN PWM GENERATOR INSTANCE
    gpio = (volatile P33C_GPIO_INSTANCE_t*)((volatile struct P33C_GPIO_INSTANCE_t*) ((volatile uint8_t*) &ANSELA + ((gpio_Instance - 1) * GPIO_SFR_OFFSET)));
    dac  = (volatile P33C_DAC_INSTANCE_t*) ((volatile struct P33C_GPIO_INSTANCE_t*) ((volatile uint8_t*) &DAC1CONL + ((dac_Instance - 1) * DAC_SFR_OFFSET)));
    pg   = (volatile P33C_PWM_INSTANCE_t*) ((volatile struct P33C_GPIO_INSTANCE_t*) ((volatile uint8_t*) &PG1CONL + ((pwm_Instance - 1) * PWM_SFR_OFFSET)));

    // WRITE GPIO CONFIGURATION OF PWM OUTPUT(S)
    gpio->LATx  &= (0xFFFF ^ (0x0001 << SEPCI_PWM_OUT_PINNO));  // Clear PWM output LOW
    gpio->TRISx &= (0xFFFF ^ (0x0001 << SEPCI_PWM_OUT_PINNO));  // Clear PWM output to OUTPUT
    gpio->CNPDx |= (0x0001 << SEPCI_PWM_OUT_PINNO); // Enable intern pull down register (PWM1H)
    
    // WRITE CONFIGURATION TO PWM GENERATOR x CONTROL REGISTERS
    pg->PGxCONL = REG_PGxCONL; // PGxCONL: PWM GENERATOR x CONTROL REGISTER LOW
    pg->PGxCONH = REG_PGxCONH; // PGxCONH: PWM GENERATOR x CONTROL REGISTER HIGH
    pg->PGxIOCONL = REG_PGxIOCONL; // PGxIOCONL: PWM GENERATOR x I/O CONTROL REGISTER LOW
    pg->PGxIOCONH = REG_PGxIOCONH; // PGxIOCONL: PWM GENERATOR x I/O CONTROL REGISTER HIGH
    pg->PGxEVTL = REG_PGxEVTL; // PGxEVTL: PWM GENERATOR x EVENT REGISTER LOW
    pg->PGxEVTH = REG_PGxEVTH; // PGxEVTH: PWM GENERATOR x EVENT REGISTER HIGH
    pg->PGxCLPCIL = REG_PGxCLPCIL; // PGxLCPCIL: PWM GENERATOR x CURRENT LIMIT PCI REGISTER LOW
    pg->PGxCLPCIH = REG_PGxCLPCIH; // PGxLCPCIL: PWM GENERATOR x CURRENT LIMIT PCI REGISTER HIGH
    pg->PGxFPCIL = REG_PGxFPCIL; // PGxFPCIL: PWM GENERATOR x FAULT PCI REGISTER LOW
    pg->PGxFPCIH = REG_PGxFPCIH; // PGxFPCIL: PWM GENERATOR x FAULT PCI REGISTER HIGH
    pg->PGxFFPCIL = REG_PGxFFPCIL; // PGxFFPCIL: PWM GENERATOR x FEED FORWARD PCI REGISTER LOW
    pg->PGxFFPCIH = REG_PGxFFPCIH; // PGxFFPCIL: PWM GENERATOR x FEED FORWARD PCI REGISTER HIGH
    pg->PGxSPCIL = REG_PGxSPCIL; // PGxSPCIL: PWM GENERATOR x SOFTWARE PCI REGISTER LOW
    pg->PGxSPCIH = REG_PGxSPCIH; // PGxSPCIL: PWM GENERATOR x SOFTWARE PCI REGISTER HIGH
    pg->PGxLEBL = SEPIC_LEB_PERIOD; // PGxLEBL: PWM GENERATOR x LEADING-EDGE BLANKING REGISTER LOW
    pg->PGxLEBH = REG_PGxLEBH; // PGxLEBL: PWM GENERATOR x LEADING-EDGE BLANKING REGISTER HIGH

    // PWM Generator Timing Settings
    pg->PGxPHASE = SEPIC_PWM_PHASE_SHIFT; // PGxPHASE: PWM GENERATOR x PHASE REGISTER
    pg->PGxDC = SEPIC_MAX_DUTY_CYCLE; // PGxDC: PWM GENERATOR x DUTY CYCLE REGISTER
    pg->PGxDCA = REG_PGxDCA; // PGxDCA: PWM GENERATOR x DUTY CYCLE ADJUSTMENT REGISTER
    pg->PGxPER = SEPIC_PWM_PERIOD; // PGxPER: PWM GENERATOR x PERIOD REGISTER
    pg->PGxTRIGA = SLOPE_START_DELAY; // PGxTRIGA: PWM GENERATOR x TRIGGER A REGISTER
    pg->PGxTRIGB = SLOPE_STOP_DELAY; // PGxTRIGB: PWM GENERATOR x TRIGGER B REGISTER
    pg->PGxTRIGC = 0; // PGxTRIGC: PWM GENERATOR x TRIGGER C REGISTER
    pg->PGxDTL = SEPIC_PWM_DEAD_TIME_FALLING; // PGxDTL: PWM GENERATOR x DEAD-TIME REGISTER LOW
    pg->PGxDTH = SEPIC_PWM_DEAD_TIME_RISING; // PGxDTH: PWM GENERATOR x DEAD-TIME REGISTER HIGH
    pg->PGxLEBL = SEPIC_LEB_PERIOD; // PWM GENERATOR x LEADING-EDGE BLANKING REGISTER LOW 
    pg->PGxDTL = SEPIC_PWM_DEAD_TIME_FALLING;    // PGxDTL: PWM GENERATOR x DEAD-TIME REGISTER LOW        
    pg->PGxDTH = SEPIC_PWM_DEAD_TIME_RISING;    // PGxDTH: PWM GENERATOR x DEAD-TIME REGISTER HIGH
    //pg->PG1CAP = 0x0000;   // PGxCAP: PWM GENERATOR x CAPTURE REGISTER (read only)

    // WRITE CONFIGURATION OF DIGITAL-TO-ANALOG CONVERTER / HIGH-SPEED ANALOG COMPARATOR
    dac->DACxCONL = REG_DACxCONL;       // DACxCONL: DACx CONTROL LOW REGISTER
    dac->DACxCONH = SEPIC_PCMC_DAC_CBP; // DACxCONH: DACx CONTROL HIGH REGISTER
    dac->DACxDATL = SEPIC_PCMC_DAC_MIN; // Minimum DAC level
    dac->DACxDATH = SEPIC_PCMC_DAC_MAX; // Operating DAC level (reset to zero/min)
    dac->SLPxCONL = REG_SLPxCONL;       // SLPxCONL: DACx SLOPE CONTROL REGISTER LOW
    dac->SLPxCONH = REG_SLPxCONH;       // SLPxCONH: DACx SLOPE CONTROL REGISTER HIGH
    dac->SLPxDAT = SLOPE_SLEW_RATE;     // SLPxDAT: DACx SLOPE DATA REGISTER
    
    return (1);
}

volatile uint16_t launch_sepic_pwm(volatile uint16_t pInstance) {

    volatile P33C_PWM_INSTANCE_t* pg;

    // CAPTURE MEMORY ADDRESS OF GIVEN PWM GENERATOR INSTANCE
    pg = (volatile P33C_PWM_INSTANCE_t*)((volatile struct P33C_PWM_INSTANCE_t*) ((volatile uint8_t*) & PG1CONL + ((pInstance - 1) * PWM_SFR_OFFSET)));

    pg->PGxCONL |= 0x8000; // PWM Generator Enable: PWM Generator is enabled
    pg->PGxSTAT |= 0x0008; // Update all PWM registers

    pg->PGxIOCONH |= 0x0008; // PWMxH Output Port Enable: PWM generator controls the PWMxH output pin

    return (1);
}

// This PWM is used only to generate synchronized ADC Trigger 1 for the sepic converter

volatile uint16_t init_sepic_trig_pwm(volatile uint16_t pInstance) {

    // PWM GENERATOR x CONTROL REGISTERS
    PG3CONLbits.ON = 0; // PWM Generator #3 Enable: PWM Generator is not enabled
    PG3CONLbits.TRGCNT = 0b000; // Trigger Count Select: PWM Generator produces one PWM cycle after triggered
    PG3CONLbits.HREN = 0; // High-Resolution mode is not enabled for PWM Generator x
    PG3CONLbits.CLKSEL = 0b01; // Clock Selection: PWM Generator uses Master clock selected by the MCLKSEL[1:0] (PCLKCON[1:0]) control bits
    PG3CONLbits.MODSEL = 0b000; // PWM Mode Selection: Independent Edge PWM mode

    PG3CONHbits.MDCSEL = 0; // Master Duty Cycle Register Selection: PWM Generator uses PGxDC register
    PG3CONHbits.MPERSEL = 1; // Master Period Register Selection: PWM Generator uses MPER register
    PG3CONHbits.MPHSEL = 0; // Master Phase Register Selection: PWM Generator uses PGxPHASE register
    PG3CONHbits.MSTEN = 0; // Master Update Enable: PWM Generator does not broadcast the UPDREQ status bit state or EOC signal
    PG3CONHbits.UPDMOD = 0b000; // PWM Buffer Update Mode Selection: SOC update
    PG3CONHbits.TRGMOD = 0; // PWM Generator Trigger Mode Selection: PWM Generator operates in single trigger mode
    PG3CONHbits.SOCS = 1; // Start-of-Cycle Selection: Trigger output selected by PG1

    // PGxIOCONH: PWM GENERATOR x I/O CONTROL REGISTER LOW
    PG3IOCONL = 0x0000;

    // PGxIOCONH: PWM GENERATOR x I/O CONTROL REGISTER HIGH
    PG3IOCONHbits.CAPSRC = 0b000; // Time Base Capture Source Selection: No hardware source selected for time base capture ? software only
    PG3IOCONHbits.DTCMPSEL = 0; // Dead-Time Compensation Selection: Dead-time compensation is controlled by PCI Sync logic
    PG3IOCONHbits.PMOD = 0b01; // PWM Generator Output Mode Selection: PWM Generator outputs operate in Complementary mode
    PG3IOCONHbits.PENH = 0; // PWMxH Output Port Enable: GPIO registers TRISx, LATx, Rxx registers control the PWMxH output pin
    PG3IOCONHbits.PENL = 0; // PWMxL Output Port Enable: GPIO registers TRISx, LATx, Rxx registers control the PWMxL output pin
    PG3IOCONHbits.POLH = 0; // PWMxH Output Port Enable: Output pin is active-high
    PG3IOCONHbits.POLL = 0; // PWMxL Output Port Enable: Output pin is active-high

    // PWM GENERATOR x STATUS REGISTER
    PG3STAT = 0x0000; // Reset to default

    // PWM GENERATOR x EVENT REGISTER LOW 
    PG3EVTLbits.ADTR1PS = 0b00000; // ADC Trigger 1 Postscaler Selection = 1:1
    PG3EVTLbits.ADTR1EN3 = 0b0; // PG1TRIGC  Compare Event is disabled as trigger source for ADC Trigger 1
    PG3EVTLbits.ADTR1EN2 = 0b0; // PG1TRIGB  Compare Event is disabled as trigger source for ADC Trigger 1
    PG3EVTLbits.ADTR1EN1 = 0b1; // PG1TRIGA  Compare Event is enabled as trigger source for ADC Trigger 1
    PG3EVTLbits.UPDTRG = 0b00; // User must set the UPDATE bit (PG1STAT<4>) manually
    PG3EVTLbits.PGTRGSEL = 0b000; // PWM Generator Trigger Output is EOC (not used in this case)

    // PWM GENERATOR x EVENT REGISTER HIGH
    PG3EVTHbits.FLTIEN = 0b0; // PCI Fault interrupt is disabled
    PG3EVTHbits.CLIEN = 0b0; // PCI Current-Limit interrupt is disabled
    PG3EVTHbits.FFIEN = 0b0; // PCI Feed-Forward interrupt is disabled
    PG3EVTHbits.SIEN = 0b0; // PCI Sync interrupt is disabled
    PG3EVTHbits.IEVTSEL = 0b11; // Interrupt Event Selection: Time base interrupts are disabled
    PG3EVTHbits.ADTR2EN3 = 0b0; // PG1TRIGC register compare event is disabled as trigger source for ADC Trigger 2
    PG3EVTHbits.ADTR2EN2 = 0b0; // PG1TRIGB register compare event is disabled as trigger source for ADC Trigger 2
    PG3EVTHbits.ADTR2EN1 = 0b0; // PG1TRIGA register compare event is disabled as trigger source for ADC Trigger 2
    PG3EVTHbits.ADTR1OFS = 0b00000; // ADC Trigger 1 offset = No offset

    // PCI function for current limitation is not used
    PG3CLPCIH = 0x0000; // PWM GENERATOR CL PCI REGISTER HIGH
    PG3CLPCIL = 0x0000; // PWM GENERATOR CL PCI REGISTER LOW

    // Reset further PCI control registers
    PG3FPCIH = 0x0000; // PWM GENERATOR F PCI REGISTER HIGH
    PG3FPCIL = 0x0000; // PWM GENERATOR F PCI REGISTER LOW
    PG3FFPCIH = 0x0000; // PWM GENERATOR FF PCI REGISTER HIGH
    PG3FFPCIL = 0x0000; // PWM GENERATOR FF PCI REGISTER LOW
    PG3SPCIH = 0x0000; // PWM GENERATOR S PCI REGISTER HIGH
    PG3SPCIL = 0x0000; // PWM GENERATOR S PCI REGISTER LOW

    // Leading edge blanking is not used
    PG3LEBH = 0x0000;
    PG3LEBL = 0x0000;


    // PGxPHASE: PWM GENERATOR x PHASE REGISTER
    PG3PHASE = 0;

    // PGxDC: PWM GENERATOR x DUTY CYCLE REGISTER
    PG3DC = 800;

    // PGxDCA: PWM GENERATOR x DUTY CYCLE ADJUSTMENT REGISTER
    PG3DCA = 0x0000;

    // PGxPER: PWM GENERATOR x PERIOD REGISTER        
    PG3PER = 0; // Master defines the period

    // PGxTRIGA: PWM GENERATOR x TRIGGER A REGISTER
    PG3TRIGA = VOUT_ADC_TRIGGER_DELAY; // ToDo: Check this value on oscilloscope

    // PGxTRIGB: PWM GENERATOR x TRIGGER B REGISTER       
    PG3TRIGB = 0;

    // PGxTRIGC: PWM GENERATOR x TRIGGER C REGISTER        
    PG3TRIGC = 0;

    // PGxDTL: PWM GENERATOR x DEAD-TIME REGISTER LOW        
    PG3DTL = 0;

    // PGxDTH: PWM GENERATOR x DEAD-TIME REGISTER HIGH
    PG3DTH = 0;

    //  PG3CAP      = 0x0000;   // Read only register

    return (1);
}

volatile uint16_t launch_sepic_trig_pwm(volatile uint16_t pInstance) {

    Nop();
    Nop();
    Nop();

    PG3CONLbits.ON = 1; // PWM Generator #3 Enable: PWM Generator is enabled
    while (PG3STATbits.UPDATE);
    PG3STATbits.UPDREQ = 1; // Update all PWM registers

    PG3IOCONHbits.PENH = 0; // PWMxH Output Port Enable: Disabled
    PG3IOCONHbits.PENL = 0; // PWMxL Output Port Enable: Disabled
    PG3IOCONLbits.OVRENH = 0; // User Override Enable for PWMxH Pin: User override disabled
    PG3IOCONLbits.OVRENL = 0; // User Override Enable for PWMxL Pin: User override disabled

    return (1);
}

