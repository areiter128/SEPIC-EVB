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
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef SEPIC_HARDWARE_DESCRIPTOR_H
#define	SEPIC_HARDWARE_DESCRIPTOR_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
#include <stdbool.h>

#include "globals.h"
#include "c2p2z_sepic.h"

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

/*!Hardware Abstraction
 * *************************************************************************************************
 * Summary:
 * Global defines for hardware specific parameters
 * 
 * Description:
 * This section is used to define hardware specific parameters such as output voltage dividers,
 * reference levels or feedback gains. Pre-compiler macros are used to translate physical  
 * values into binary (integer) numbers to be written to SFRs
 * 
 * *************************************************************************************************/

#define VOUT_NOMINAL        15.0            // Nominal output voltage

#define SEPIC_VOUT_R1       (2.0 * 2.87)    // Upper voltage divider resistor in kOhm
#define SEPIC_VOUT_R2       (1.0)           // Lower voltage divider resistor in kOhm

#define SEPIC_VOUT_FB_GAIN  (float)((SEPIC_VOUT_R2) / (SEPIC_VOUT_R1 + SEPIC_VOUT_R2))
#define SEPIC_V_OUT_REF     (uint16_t)(VOUT_NOMINAL * SEPIC_VOUT_FB_GAIN / ADC_GRAN)

/*!Startup Behavior
 * *************************************************************************************************
 * Summary:
 * Global defines for soft-start specific parameters
 * 
 * Description:
 * This section is used to define power supply startup timing setting. The soft-start sequence 
 * is part of the power controller. It allows to program specific timings for Power On Delay,
 * Ramp Period and Power Good Delay. After the startup has passed these three timing periods,
 * the power supply is ending up in "normal" operation, continuously regulating the output until 
 * a fault is detected or the operating state is changed for any other reason.
 * 
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers to 
 * be written to SFRs and variables.
 * 
 * *************************************************************************************************/

#define SEPIC_POWER_ON_DELAY          500e-3      // power on delay in [sec]
#define SEPIC_RAMP_PERIOD             50e-3         // ramp period in [sec]
#define SEPIC_POWER_GOOD_DELAY        100e-3        // power good in [sec]

#define SEPIC_POD       (uint16_t)(((float)SEPIC_POWER_ON_DELAY / (float)MAIN_EXECUTION_PERIOD)-1)
#define SEPIC_RPER      (uint16_t)(((float)SEPIC_RAMP_PERIOD / (float)MAIN_EXECUTION_PERIOD)-1)
#define SEPIC_PGD       (uint16_t)(((float)SEPIC_POWER_GOOD_DELAY / (float)MAIN_EXECUTION_PERIOD)-1)
#define SEPIC_REF_STEP  (uint16_t)((float)SEPIC_V_OUT_REF / (float)(SEPIC_RPER + 1))


// Hardware-dependent defines
#define _SEPIC_VOUT_ADCInterrupt    _ADCAN16Interrupt   
#define SEPIC_VOUT_ADCBUF           ADCBUF16
#define SEPIC_VOUT_ADCTRIG          PG3TRIGA
#define SEPIC_ADC_INPUT_OFFSET      0

#define SEPIC_PCMC_DAC              DAC1DATH    // ToDo: Remove after having created peripheral instance mapping
#define SEPIC_PCMC_DAC_CHANNEL      1           // DAC/CMP Instance Index (e.g. 1=CMP1, 2=CMP2, etc.)
#define SEPIC_PCMC_FB_INPUT         0           // Comparator input selection: 0=A, 1=B, 2=C, 3=D
#define SEPIC_PCMC_DAC_MIN          50          // Comparator input selection: 0=A, 1=B, 2=C, 3=D
#define SEPIC_PCMC_DAC_MAX          4000
#define SEPIC_PCMC_DAC_CBP          50          // Comparator Blanking Period
#define SLOPE_SLEW_RATE             43          // Compensation Slope Slew Rate Value
    
#define SEPIC_PWM_CHANNEL           1           // PWM Instance Index (e.g. 1=PWM1, 2=PWM2, etc.)
#define SEPIC_PWM_PERIOD            1429        // Measured in [tick = 2ns] -> 350 kHz 
#define SEPIC_PWM_PHASE_SHIFT       0           // Measured in [tick = 2ns] -> 350 kHz 
#define SEPIC_MAX_DUTY_CYCLE        1150        // This sets the maximum duty cycle
#define SEPIC_LEB_PERIOD            100         // Leading Edge Blanking = n x PWM resolution (here: 50 x 2ns = 100ns)
#define SEPIC_PWM_DEAD_TIME_RISING  0           // Rising edge dead time [2ns]
#define SEPIC_PWM_DEAD_TIME_FALLING 0           // Falling edge dead time [2ns]

#define VOUT_ADC_TRIGGER_DELAY      120         // With respect to the start of the PWM cycle 
#define SLOPE_START_DELAY           100         // With respect to the start of the PWM cycle; ToDo: How is this influenced by the settling and steady-state time of the DAC-slope generator?
#define SLOPE_STOP_DELAY            1150        // With respect to the start of the PWM cycle

#define SEPCI_PWM_OUT_PORT          1           // Device Port No: 0=A, 1=B, 2=C, 3=D, etc
#define SEPCI_PWM_OUT_PINNO         14          // Device Pin No:  10=Rx10 where X is the port index A, B, C, etc

    
extern volatile uint16_t init_sepic_pcmc(volatile uint16_t pwm_Instance, volatile uint16_t dac_Instance, volatile uint16_t gpio_Instance);
extern volatile uint16_t launch_sepic_pwm(volatile uint16_t pInstance);
extern volatile uint16_t init_sepic_trig_pwm(volatile uint16_t pInstance);
extern volatile uint16_t launch_sepic_trig_pwm(volatile uint16_t pInstance);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* SEPIC_HARDWARE_DESCRIPTOR_H */

