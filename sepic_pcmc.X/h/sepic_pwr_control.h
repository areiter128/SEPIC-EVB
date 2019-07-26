/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   pwr_control.h
 * Author: M91406
 * Comments: power controller functions for buck converter
 * Revision history: 
 * 1.0  initial version
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef INITIALIZE_SEPIC_POWER_CONTROL_H
#define	INITIALIZE_SEPIC_POWER_CONTROL_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
#include <stdbool.h>

#include "sepic_hwdescr.h"

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

// ==============================================================================================
// SEPIC converter operation status bits data structure and defines
// ==============================================================================================

 typedef enum {
    SEPIC_STAT_OFF     = 0b000,  // Converter Status Off: Everything is inactive incl. peripherals
    SEPIC_STAT_STANDBY = 0b001,  // Converter Status Standby: Peripherals are running but controller and PWM outputs are off
    SEPIC_STAT_START   = 0b010,  // Converter Status Startup: Converter is executing its startup procedure
    SEPIC_STAT_ON      = 0b011,  // Converter Status Active and Running
    SEPIC_STAT_FAULT   = 0b100   // Converter Status FAULT: Power supply has been shut down waiting for restart attempt
}SEPIC_CONVERTER_OP_STATUS_e;

typedef struct {
    volatile SEPIC_CONVERTER_OP_STATUS_e op_status :3;  // Bit <0:2> operation status
    volatile unsigned : 7;                              // Bit <3:9> (reserved)
    volatile bool pwm_active  :1;                       // Bit 10: Status bit indicating that the PWM outputs have been enabled
    volatile bool adc_active  :1;                       // Bit 11: Status bit indicating that the ADC has been started and is sampling data
    volatile bool fault_active  :1;                     // Bit 12: Status bit indicating that a critical fault condition has been detected
    volatile bool GO :1;                                // Bit 13: POWER SUPPLY START bit (will trigger startup procedure when set)
    volatile bool auto_start :1;                        // Bit 14: Auto-Start will automatically enable the converter and set the GO bit when ready
    volatile bool enabled :1;                           // Bit 15: Enable-bit (when disabled, power supply will reset in STANDBY mode)
}__attribute__((packed))SEPIC_CONVERTER_STATUS_FLAGS_t;

typedef union {
	volatile uint16_t value;                 // buffer for 16-bit word read/write operations
	volatile SEPIC_CONVERTER_STATUS_FLAGS_t flags; // data structure for single bit addressing operations
} SEPIC_CONVERTER_STATUS_t;                  // SEPIC operation status bits

// ==============================================================================================
// SEPIC converter soft-start settings data structure and defines
// ==============================================================================================
typedef enum {
    SEPIC_SS_INIT            = 0,  // Soft-Start Phase Initialization
    SEPIC_SS_LAUNCH_PER      = 1,  // Soft-Start Phase Peripheral Launch
    SEPIC_SS_STANDBY         = 2,  // Soft-Start Phase Standby (wait for GO command)
    SEPIC_SS_PWR_ON_DELAY    = 3,  // Soft-Start Phase Power On Delay
    SEPIC_SS_RAMP_UP         = 4,  // Soft-Start Phase Output Ramp Up 
    SEPIC_SS_PWR_GOOD_DELAY  = 5,  // Soft-Start Phase Power Good Delay
    SEPIC_SS_COMPLETE        = 6   // Soft-Start Phase Complete
}SEPIC_SOFT_START_STATUS_e;

typedef struct {
    volatile uint16_t reference;            // Soft-Start target reference value
    volatile uint16_t pwr_on_delay;         // Soft-Start POwer On Delay
    volatile uint16_t precharge_delay;      // Soft-Start Bootstrap Capacitor pre-charge delay
    volatile uint16_t ramp_period;          // Soft-Start Ramp-Up Duration
    volatile uint16_t ramp_ref_increment;   // Soft-Start Single Reference Increment per Step
    volatile uint16_t pwr_good_delay;       // Soft-Start Power Good Delay
    volatile uint16_t counter;              // Soft-Start Execution Counter
    volatile uint16_t phase;                // Soft-Start Phase Index
}SEPIC_SOFT_START_t;                        // SEPIC soft-start settings and variables

// ==============================================================================================
// SEPIC converter soft-start settings data structure and defines
// ==============================================================================================

typedef struct {
    volatile uint16_t i_out;    // SEPIC output current
    volatile uint16_t v_in;     // SEPIC input voltage
    volatile uint16_t v_out;    // SEPIC output voltage
    volatile uint16_t v_ref;    // SEPIC reference voltage
}SEPIC_CONVERTER_DATA_t;        // SEPIC runtime data

typedef struct {
    volatile SEPIC_CONVERTER_STATUS_t status; // SEPIC operation status bits
    volatile SEPIC_SOFT_START_t soft_start;   // SEPIC soft-start settings and variables
    volatile SEPIC_CONVERTER_DATA_t data;     // SEPIC runtime data
}SEPIC_POWER_CONTROLLER_t;                    // SEPIC control & monitoring data structure



// ==============================================================================================
// SEPIC converter Peripheral Configuration for Peak Current Mode Control (PCMC)
// ==============================================================================================

/* PGxCONL: PWM GENERATOR x CONTROL REGISTER LOW

                           ________________ BIT 15: ON: Enable: PWM Generator is not enabled
                          | _______________ BIT 14: (reserved) 
                          || ______________ BIT 13: (unimplemented) 
                          ||| _____________ BIT 12: (unimplemented) 
                          |||| ____________ BIT 11: (unimplemented) 
                          ||||| ___________ BIT 10: TRGCNT[2:0]: Trigger Count Selection: PWM Generator produces one PWM cycle after triggered
                          |||||| __________ BIT  9: 
                          ||||||| _________ BIT  8: 
                          |||||||| ________ BIT  7: HREN: PWM Generator x High-Resolution Enable: PWM Generator x operates in standard resolution
                          ||||||||| _______ BIT  6: (unimplemented)
                          |||||||||| ______ BIT  5: (unimplemented)
                          ||||||||||| _____ BIT  4: CLKSEL[1:0]: Clock Selection: 01 = PWM Generator uses Master clock selected by the MCLKSEL[1:0] (PCLKCON[1:0]) control bits
                          |||||||||||| ____ BIT  3: 
                          ||||||||||||| ___ BIT  2: MODSEL[2:0]: Mode Selection: Independent Edge PWM mode
                          |||||||||||||| __ BIT  1: 
                          ||||||||||||||| _ BIT  0: 
                          ||||||||||||||||  */
#define REG_PGxCONL     0b0000000000001000
    
/* PGxCONH: PWM GENERATOR x CONTROL REGISTER HIGH

                           ________________ BIT 15: MDCSEL: Master Duty Cycle Register Selection: 0 = PWM Generator uses PGxDC register
                          | _______________ BIT 14: MPERSEL: Master Period Register Selection: 1 = PWM Generator uses MPER register
                          || ______________ BIT 13: MPHSEL: Master Phase Register Selection: 0 = PWM Generator uses PGxPHASE register
                          ||| _____________ BIT 12: (unimplemented) 
                          |||| ____________ BIT 11: MSTEN: Master Update Enable: 0 = PWM Generator does not broadcast the UPDREQ status bit state or EOC signal
                          ||||| ___________ BIT 10: UPDMOD[2:0]: PWM Buffer Update Mode Selection: 001 = Immediate update
                          |||||| __________ BIT  9: 
                          ||||||| _________ BIT  8: 
                          |||||||| ________ BIT  7: (reserved)
                          ||||||||| _______ BIT  6: TRGMOD: PWM Generator Trigger Mode Selection: PWM Generator operates in Retriggerable mode
                          |||||||||| ______ BIT  5: (unimplemented)
                          ||||||||||| _____ BIT  4: (unimplemented)
                          |||||||||||| ____ BIT  3: SOCS[3:0]: Start-of-Cycle Selection: Local EOC ? PWM Generator is self-triggered
                          ||||||||||||| ___ BIT  2: 
                          |||||||||||||| __ BIT  1: 
                          ||||||||||||||| _ BIT  0: 
                          ||||||||||||||||  */
#define REG_PGxCONH     0b0100000101000000


/* PGxIOCONL: PWM GENERATOR x I/O CONTROL REGISTER LOW

                           ________________ BIT 15: CLMOD: Current-Limit Mode Selection
                          | _______________ BIT 14: SWAP: Swap PWM Signals to PWMxH and PWMxL Device Pins
                          || ______________ BIT 13: OVRENH: User Override Enable for PWMxH Pin
                          ||| _____________ BIT 12: OVRENL: User Override Enable for PWMxL Pin
                          |||| ____________ BIT 11: OVRDAT[1:0]: Data for PWMxH/PWMxL Pins if Override is Enabled
                          ||||| ___________ BIT 10: 
                          |||||| __________ BIT  9: OSYNC[1:0]: User Output Override Synchronization Control
                          ||||||| _________ BIT  8: 
                          |||||||| ________ BIT  7: FLTDAT[1:0]: Data for PWMxH/PWMxL Pins if Fault Event is Active
                          ||||||||| _______ BIT  6: 
                          |||||||||| ______ BIT  5: CLDAT[1:0]: Data for PWMxH/PWMxL Pins if Current-Limit Event is Active
                          ||||||||||| _____ BIT  4: 
                          |||||||||||| ____ BIT  3: FFDAT[1:0]: Data for PWMxH/PWMxL Pins if Feed-Forward Event is Active
                          ||||||||||||| ___ BIT  2: 
                          |||||||||||||| __ BIT  1: DBDAT[1:0]: Data for PWMxH/PWMxL Pins if Debug Mode is Active
                          ||||||||||||||| _ BIT  0: 
                          ||||||||||||||||  */
#define REG_PGxIOCONL   0b0011000000010000

   
/* PGxIOCONH: PWM GENERATOR x I/O CONTROL REGISTER HIGH

                           ________________ BIT 15: (unimplemented)
                          | _______________ BIT 14: CAPSRC[2:0]: Time Base Capture Source Selection
                          || ______________ BIT 13: 
                          ||| _____________ BIT 12: 
                          |||| ____________ BIT 11: (unimplemented)
                          ||||| ___________ BIT 10: (unimplemented)
                          |||||| __________ BIT  9: (unimplemented)
                          ||||||| _________ BIT  8: DTCMPSEL: Dead-Time Compensation Select
                          |||||||| ________ BIT  7: (unimplemented)
                          ||||||||| _______ BIT  6: (unimplemented)
                          |||||||||| ______ BIT  5: PMOD[1:0]: PWM Generator Output Mode Selection
                          ||||||||||| _____ BIT  4: 
                          |||||||||||| ____ BIT  3: PENH: PWMxH Output Port Enable
                          ||||||||||||| ___ BIT  2: PENL: PWMxL Output Port Enable
                          |||||||||||||| __ BIT  1: POLH: PWMxH Output Polarity
                          ||||||||||||||| _ BIT  0: POLL: PWMxL Output Polarity
                          ||||||||||||||||  */
#define REG_PGxIOCONH   0b0000000000010000

/* PGxEVTL: PWM GENERATOR x EVENT REGISTER LOW

                           ________________ BIT 15: ADTR1PS[4:0]: ADC Trigger 1 Postscaler Selection
                          | _______________ BIT 14: 
                          || ______________ BIT 13: 
                          ||| _____________ BIT 12: 
                          |||| ____________ BIT 11: 
                          ||||| ___________ BIT 10: ADTR1EN3: ADC Trigger 1 Source is PGxTRIGC Compare Event Enable
                          |||||| __________ BIT  9: ADTR1EN2: ADC Trigger 1 Source is PGxTRIGB Compare Event Enable
                          ||||||| _________ BIT  8: ADTR1EN1: ADC Trigger 1 Source is PGxTRIGA Compare Event Enable
                          |||||||| ________ BIT  7: (unimplemented)
                          ||||||||| _______ BIT  6: (unimplemented)
                          |||||||||| ______ BIT  5: (unimplemented)
                          ||||||||||| _____ BIT  4: UPDTRG[1:0]: Update Trigger Selection
                          |||||||||||| ____ BIT  3: 
                          ||||||||||||| ___ BIT  2: PGTRGSEL[2:0]: PWM Generator Trigger Output Selection
                          |||||||||||||| __ BIT  1: 
                          ||||||||||||||| _ BIT  0: 
                          ||||||||||||||||  */
#define REG_PGxEVTL     0b0000000100000000


/* PGxEVTH: PWM GENERATOR x EVENT REGISTER HIGH

                           ________________ BIT 15: FLTIEN: PCI Fault Interrupt Enable
                          | _______________ BIT 14: CLIEN: PCI Current-Limit Interrupt Enable
                          || ______________ BIT 13: FFIEN: PCI Feed-Forward Interrupt Enable
                          ||| _____________ BIT 12: SIEN: PCI Sync Interrupt Enable
                          |||| ____________ BIT 11: (unimplemented)
                          ||||| ___________ BIT 10: (unimplemented)
                          |||||| __________ BIT  9: IEVTSEL[1:0]: Interrupt Event Selection
                          ||||||| _________ BIT  8: 
                          |||||||| ________ BIT  7: ADTR2EN3: ADC Trigger 2 Source is PGxTRIGC Compare Event Enable
                          ||||||||| _______ BIT  6: ADTR2EN2: ADC Trigger 2 Source is PGxTRIGB Compare Event Enable
                          |||||||||| ______ BIT  5: ADTR2EN1: ADC Trigger 2 Source is PGxTRIGA Compare Event Enable
                          ||||||||||| _____ BIT  4: ADTR1OFS[4:0]: ADC Trigger 1 Offset Selection
                          |||||||||||| ____ BIT  3: 
                          ||||||||||||| ___ BIT  2: 
                          |||||||||||||| __ BIT  1: 
                          ||||||||||||||| _ BIT  0: 
                          ||||||||||||||||  */
#define REG_PGxEVTH     0b0000001101000000


/* PGxCLPCIL: PWM GENERATOR CURRENT LIMIT PCI REGISTER LOW

                           ________________ BIT 15: TSYNCDIS: Termination Synchronization Disable
                          | _______________ BIT 14: TERM[2:0]: Termination Event Selection
                          || ______________ BIT 13: 
                          ||| _____________ BIT 12: 
                          |||| ____________ BIT 11: AQPS: Acceptance Qualifier Polarity Selection
                          ||||| ___________ BIT 10: AQSS[2:0]: Acceptance Qualifier Source Selection
                          |||||| __________ BIT  9: 
                          ||||||| _________ BIT  8: 
                          |||||||| ________ BIT  7: SWTERM: PCI Software Termination
                          ||||||||| _______ BIT  6: PSYNC: PCI Synchronization Control
                          |||||||||| ______ BIT  5: PPS: PCI Polarity Selection
                          ||||||||||| _____ BIT  4: PSS[4:0]: PCI Source Selection
                          |||||||||||| ____ BIT  3: 
                          ||||||||||||| ___ BIT  2: 
                          |||||||||||||| __ BIT  1: 
                          ||||||||||||||| _ BIT  0:                
                          ||||||||||||||||  */
#define REG_PGxCLPCIL   0b0001101000011011
    
/* PGxCLPCIH: PWM GENERATOR CURRENT LIMIT PCI REGISTER HIGH

                           ________________ BIT 15: BPEN: PCI Bypass Enable
                          | _______________ BIT 14: BPSEL[2:0]: PCI Bypass Source Selection
                          || ______________ BIT 13: 
                          ||| _____________ BIT 12: 
                          |||| ____________ BIT 11: (unimplemented)
                          ||||| ___________ BIT 10: ACP[2:0]: PCI Acceptance Criteria Selection
                          |||||| __________ BIT  9: 
                          ||||||| _________ BIT  8: 
                          |||||||| ________ BIT  7: SWPCI: Software PCI Control
                          ||||||||| _______ BIT  6: SWPCIM[1:0]: Software PCI Control Mode
                          |||||||||| ______ BIT  5: 
                          ||||||||||| _____ BIT  4: LATMOD: PCI SR Latch Mode
                          |||||||||||| ____ BIT  3: TQPS: Termination Qualifier Polarity Selection
                          ||||||||||||| ___ BIT  2: TQSS[2:0]: Termination Qualifier Source Selection
                          |||||||||||||| __ BIT  1: 
                          ||||||||||||||| _ BIT  0: 
                          ||||||||||||||||  */
#define REG_PGxCLPCIH   0b0000011000000000

/* PGxFPCIL: PWM GENERATOR FAULT PCI REGISTER LOW

                           ________________ BIT 15: TSYNCDIS: Termination Synchronization Disable
                          | _______________ BIT 14: TERM[2:0]: Termination Event Selection
                          || ______________ BIT 13: 
                          ||| _____________ BIT 12: 
                          |||| ____________ BIT 11: AQPS: Acceptance Qualifier Polarity Selection
                          ||||| ___________ BIT 10: AQSS[2:0]: Acceptance Qualifier Source Selection
                          |||||| __________ BIT  9: 
                          ||||||| _________ BIT  8: 
                          |||||||| ________ BIT  7: SWTERM: PCI Software Termination
                          ||||||||| _______ BIT  6: PSYNC: PCI Synchronization Control
                          |||||||||| ______ BIT  5: PPS: PCI Polarity Selection
                          ||||||||||| _____ BIT  4: PSS[4:0]: PCI Source Selection
                          |||||||||||| ____ BIT  3: 
                          ||||||||||||| ___ BIT  2: 
                          |||||||||||||| __ BIT  1: 
                          ||||||||||||||| _ BIT  0: 
                          ||||||||||||||||  */
#define REG_PGxFPCIL    0b0000000000000000
    
/* PGxFPCIH: PWM GENERATOR FAULT PCI REGISTER HIGH

                           ________________ BIT 15: BPEN: PCI Bypass Enable
                          | _______________ BIT 14: BPSEL[2:0]: PCI Bypass Source Selection
                          || ______________ BIT 13: 
                          ||| _____________ BIT 12: 
                          |||| ____________ BIT 11: (unimplemented)
                          ||||| ___________ BIT 10: ACP[2:0]: PCI Acceptance Criteria Selection
                          |||||| __________ BIT  9: 
                          ||||||| _________ BIT  8: 
                          |||||||| ________ BIT  7: SWPCI: Software PCI Control
                          ||||||||| _______ BIT  6: SWPCIM[1:0]: Software PCI Control Mode
                          |||||||||| ______ BIT  5: 
                          ||||||||||| _____ BIT  4: LATMOD: PCI SR Latch Mode
                          |||||||||||| ____ BIT  3: TQPS: Termination Qualifier Polarity Selection
                          ||||||||||||| ___ BIT  2: TQSS[2:0]: Termination Qualifier Source Selection
                          |||||||||||||| __ BIT  1: 
                          ||||||||||||||| _ BIT  0:               
                          ||||||||||||||||  */
#define REG_PGxFPCIH    0b0000000000000000

/* PGxFFPCIL: PWM GENERATOR FEED FORWARD PCI REGISTER LOW

                           ________________ BIT 15: TSYNCDIS: Termination Synchronization Disable
                          | _______________ BIT 14: TERM[2:0]: Termination Event Selection
                          || ______________ BIT 13: 
                          ||| _____________ BIT 12: 
                          |||| ____________ BIT 11: AQPS: Acceptance Qualifier Polarity Selection
                          ||||| ___________ BIT 10: AQSS[2:0]: Acceptance Qualifier Source Selection
                          |||||| __________ BIT  9: 
                          ||||||| _________ BIT  8: 
                          |||||||| ________ BIT  7: SWTERM: PCI Software Termination
                          ||||||||| _______ BIT  6: PSYNC: PCI Synchronization Control
                          |||||||||| ______ BIT  5: PPS: PCI Polarity Selection
                          ||||||||||| _____ BIT  4: PSS[4:0]: PCI Source Selection
                          |||||||||||| ____ BIT  3: 
                          ||||||||||||| ___ BIT  2: 
                          |||||||||||||| __ BIT  1: 
                          ||||||||||||||| _ BIT  0: 
                          ||||||||||||||||  */
#define REG_PGxFFPCIL   0b0000000000000000
    
/* PGxFFPCIH: PWM GENERATOR FEED FORWARD PCI REGISTER HIGH

                           ________________ BIT 15: BPEN: PCI Bypass Enable
                          | _______________ BIT 14: BPSEL[2:0]: PCI Bypass Source Selection
                          || ______________ BIT 13: 
                          ||| _____________ BIT 12: 
                          |||| ____________ BIT 11: (unimplemented)
                          ||||| ___________ BIT 10: ACP[2:0]: PCI Acceptance Criteria Selection
                          |||||| __________ BIT  9: 
                          ||||||| _________ BIT  8: 
                          |||||||| ________ BIT  7: SWPCI: Software PCI Control
                          ||||||||| _______ BIT  6: SWPCIM[1:0]: Software PCI Control Mode
                          |||||||||| ______ BIT  5: 
                          ||||||||||| _____ BIT  4: LATMOD: PCI SR Latch Mode
                          |||||||||||| ____ BIT  3: TQPS: Termination Qualifier Polarity Selection
                          ||||||||||||| ___ BIT  2: TQSS[2:0]: Termination Qualifier Source Selection
                          |||||||||||||| __ BIT  1: 
                          ||||||||||||||| _ BIT  0:               
                          ||||||||||||||||  */
#define REG_PGxFFPCIH   0b0000000000000000
    
/* PGxSPCIL: PWM GENERATOR SOFTWARE PCI REGISTER LOW

                           ________________ BIT 15: TSYNCDIS: Termination Synchronization Disable
                          | _______________ BIT 14: TERM[2:0]: Termination Event Selection
                          || ______________ BIT 13: 
                          ||| _____________ BIT 12: 
                          |||| ____________ BIT 11: AQPS: Acceptance Qualifier Polarity Selection
                          ||||| ___________ BIT 10: AQSS[2:0]: Acceptance Qualifier Source Selection
                          |||||| __________ BIT  9: 
                          ||||||| _________ BIT  8: 
                          |||||||| ________ BIT  7: SWTERM: PCI Software Termination
                          ||||||||| _______ BIT  6: PSYNC: PCI Synchronization Control
                          |||||||||| ______ BIT  5: PPS: PCI Polarity Selection
                          ||||||||||| _____ BIT  4: PSS[4:0]: PCI Source Selection
                          |||||||||||| ____ BIT  3: 
                          ||||||||||||| ___ BIT  2: 
                          |||||||||||||| __ BIT  1: 
                          ||||||||||||||| _ BIT  0: 
                          ||||||||||||||||  */
#define REG_PGxSPCIL    0b0000000000000000
    
/* PGxSPCIH: PWM GENERATOR SOFTWARE PCI REGISTER HIGH

                           ________________ BIT 15: BPEN: PCI Bypass Enable
                          | _______________ BIT 14: BPSEL[2:0]: PCI Bypass Source Selection
                          || ______________ BIT 13: 
                          ||| _____________ BIT 12: 
                          |||| ____________ BIT 11: (unimplemented)
                          ||||| ___________ BIT 10: ACP[2:0]: PCI Acceptance Criteria Selection
                          |||||| __________ BIT  9: 
                          ||||||| _________ BIT  8: 
                          |||||||| ________ BIT  7: SWPCI: Software PCI Control
                          ||||||||| _______ BIT  6: SWPCIM[1:0]: Software PCI Control Mode
                          |||||||||| ______ BIT  5: 
                          ||||||||||| _____ BIT  4: LATMOD: PCI SR Latch Mode
                          |||||||||||| ____ BIT  3: TQPS: Termination Qualifier Polarity Selection
                          ||||||||||||| ___ BIT  2: TQSS[2:0]: Termination Qualifier Source Selection
                          |||||||||||||| __ BIT  1: 
                          ||||||||||||||| _ BIT  0:               
                          ||||||||||||||||  */
#define REG_PGxSPCIH    0b0000000000000000

/* PGxLEBH: PWM GENERATOR x LEADING-EDGE BLANKING REGISTER HIGH

                           ________________ BIT 15: (unimplemented)
                          | _______________ BIT 14: (unimplemented)
                          || ______________ BIT 13: (unimplemented)
                          ||| _____________ BIT 12: (unimplemented)
                          |||| ____________ BIT 11: (unimplemented)
                          ||||| ___________ BIT 10: PWMPCI[2:0]: PWM Source for PCI Selection
                          |||||| __________ BIT  9: 
                          ||||||| _________ BIT  8: 
                          |||||||| ________ BIT  7: (unimplemented)
                          ||||||||| _______ BIT  6: (unimplemented)
                          |||||||||| ______ BIT  5: (unimplemented)
                          ||||||||||| _____ BIT  4: (unimplemented)
                          |||||||||||| ____ BIT  3: PHR: PWMxH Rising Edge Trigger Enable
                          ||||||||||||| ___ BIT  2: PHF: PWMxH Falling Edge Trigger Enable
                          |||||||||||||| __ BIT  1: PLR: PWMxL Rising Edge Trigger Enable
                          ||||||||||||||| _ BIT  0: PLF: PWMxL Falling Edge Trigger Enable
                          ||||||||||||||||  */
#define REG_PGxLEBH     0b0000000000000000

/* PGxDCA: PWM GENERATOR x DUTY CYCLE ADJUSTMENT REGISTER

                           ________________ BIT 15: PGxDCA[15:0]: PWM Generator x Duty Cycle Adjustment Register
                          | _______________ BIT 14: 
                          || ______________ BIT 13: 
                          ||| _____________ BIT 12: 
                          |||| ____________ BIT 11: 
                          ||||| ___________ BIT 10: 
                          |||||| __________ BIT  9: 
                          ||||||| _________ BIT  8: 
                          |||||||| ________ BIT  7: 
                          ||||||||| _______ BIT  6: 
                          |||||||||| ______ BIT  5: 
                          ||||||||||| _____ BIT  4: 
                          |||||||||||| ____ BIT  3: 
                          ||||||||||||| ___ BIT  2: 
                          |||||||||||||| __ BIT  1: 
                          ||||||||||||||| _ BIT  0: 
                          ||||||||||||||||  */
#define REG_PGxDCA      0b0000000000000000


/* DACxCONL: DACx CONTROL REGISTER LOW

                           ________________ BIT 15: DACEN: Individual DACx Module Enable
                          | _______________ BIT 14: IRQM[1:0]: Interrupt Mode select
                          || ______________ BIT 13: 
                          ||| _____________ BIT 12: (unimplemented)
                          |||| ____________ BIT 11: (unimplemented)
                          ||||| ___________ BIT 10: CBE: Comparator Blank Enable
                          |||||| __________ BIT  9: DACOEN: DACx Output Buffer Enable
                          ||||||| _________ BIT  8: FLTREN: Comparator Digital Filter Enable
                          |||||||| ________ BIT  7: CMPSTAT: Comparator Status
                          ||||||||| _______ BIT  6: CMPPOL: Comparator Output Polarity Control
                          |||||||||| ______ BIT  5: INSEL[2:0]: Comparator Input Source Select
                          ||||||||||| _____ BIT  4: 
                          |||||||||||| ____ BIT  3: 
                          ||||||||||||| ___ BIT  2: HYSPOL: Comparator Hysteresis Polarity Select
                          |||||||||||||| __ BIT  1: HYSSEL[1:0]: Comparator Hysteresis Select
                          ||||||||||||||| _ BIT  0: 
                          ||||||||||||||||  */
#define REG_DACxCONL    0b0000010100000101

/* DACxCONH: DACx CONTROL REGISTER HIGH => Timing Register: User value will be set in code */
/* DACxCONL: DACx CONTROL REGISTER LOW  => Timing Register: User value will be set in code */
  
/* SLPxCONL: DACx SLOPE CONTROL REGISTER LOW

                           ________________ BIT 15: HCFSEL[3:0]: Hysteretic Comparator Function Input Selection
                          | _______________ BIT 14: 
                          || ______________ BIT 13: 
                          ||| _____________ BIT 12: 
                          |||| ____________ BIT 11: SLPSTOPA[3:0]: Slope Stop A Signal Selection
                          ||||| ___________ BIT 10: 
                          |||||| __________ BIT  9: 
                          ||||||| _________ BIT  8: 
                          |||||||| ________ BIT  7: SLPSTOPB[3:0]: Slope Stop B Signal Selection
                          ||||||||| _______ BIT  6: 
                          |||||||||| ______ BIT  5: 
                          ||||||||||| _____ BIT  4: 
                          |||||||||||| ____ BIT  3: SLPSTRT[3:0]: Slope Start Signal Selection
                          ||||||||||||| ___ BIT  2: 
                          |||||||||||||| __ BIT  1: 
                          ||||||||||||||| _ BIT  0: 
                          ||||||||||||||||  */
#define REG_SLPxCONL    0b0000000100010001

/* SLPxCONH: DACx SLOPE CONTROL REGISTER HIGH

                           ________________ BIT 15: SLOPEN: Slope Function Enable/On
                          | _______________ BIT 14: (unimplemented)
                          || ______________ BIT 13: (unimplemented)
                          ||| _____________ BIT 12: (unimplemented)
                          |||| ____________ BIT 11: HME: Hysteretic Mode Enable bit
                          ||||| ___________ BIT 10: TWME: Triangle Wave Mode Enable
                          |||||| __________ BIT  9: PSE: Positive Slope Mode Enable
                          ||||||| _________ BIT  8: (unimplemented)
                          |||||||| ________ BIT  7: (unimplemented)
                          ||||||||| _______ BIT  6: (unimplemented)
                          |||||||||| ______ BIT  5: (unimplemented)
                          ||||||||||| _____ BIT  4: (unimplemented)
                          |||||||||||| ____ BIT  3: (unimplemented)
                          ||||||||||||| ___ BIT  2: (unimplemented)
                          |||||||||||||| __ BIT  1: (unimplemented)
                          ||||||||||||||| _ BIT  0: (unimplemented)
                          ||||||||||||||||  */
#define REG_SLPxCONH    0b1000000000000000

                           
/* SLPxDAT: DACx SLOPE DATA REGISTER => Timing Register: User value will be set in code */



// ==============================================================================================
// SEPIC converter public function prototypes
// ==============================================================================================

extern volatile uint16_t init_sepic_pwr_control(volatile SEPIC_POWER_CONTROLLER_t* sepicInstance);
extern volatile uint16_t launch_sepic_pwr_control(volatile SEPIC_POWER_CONTROLLER_t* sepicInstance);
extern volatile uint16_t exec_sepic_pwr_control(volatile SEPIC_POWER_CONTROLLER_t* sepicInstance);



#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* INITIALIZE_SEPIC_POWER_CONTROL_H */

