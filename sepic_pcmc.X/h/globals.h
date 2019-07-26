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
 * File:   globals.h
 * Author: M91406
 * Comments: global defines of this application
 * Revision history: 
 * v1.0 initial version
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef APPLICATION_GLOBALS_HEADER_H
#define	APPLICATION_GLOBALS_HEADER_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// List of user included header files
#include "init/init_fosc.h"
#include "init/init_timer1.h"
#include "init/init_gpio.h"

#include "init/init_acmp.h"
#include "init/init_adc.h"
#include "init/init_pwm.h"

#include "sepic_pwr_control.h"
#include "ext_reference.h"


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

/*!SEPIC_POWER_CONTROLLER_t data structure SEPIC
 * *************************************************************************************************
 * Summary:
 * Global data object for the SEPIC 
 * 
 * Description:
 * the 'sepic' data object holds all status, control and monitoring values of the SEPIC power 
 * controller. The SEPIC_POWER_CONTROLLER_t data structure is defined in sepic_pwr_contro.h.
 * Please refer to the comments on top of this file for further information.
 *  
 * *************************************************************************************************/
extern volatile SEPIC_POWER_CONTROLLER_t sepic;

/*!Microcontroller Abstraction
 * *************************************************************************************************
 * Summary:
 * Global defines for device specific parameters
 * 
 * Description:
 * This section is used to define device specific parameters like ADC reference and
 * resolution. Pre-compiler macros are used to translate physical values into binary 
 * (integer) numbers to be written to SFRs
 * 
 * *************************************************************************************************/
#define CPU_FREQUENCY        100000000  // CPU frequency in [Hz]
#define ADC_REF              3.300 // ADC reference voltage in V
#define ADC_RES              12.0  // ADC resolution in [bit]
#define ADC_GRAN             (float)(ADC_REF / pow(2.0, ADC_RES)) // ADC granularity in [V/tick]
    
/*!State Machine Settings
 * *************************************************************************************************
 * Summary:
 * Global defines for state-machine specific parameters
 * 
 * Description:
 * This section is used to define state-machine settings such as the main execution call interval. 
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers to be 
 * written to SFRs and variables.
 * 
 * *************************************************************************************************/
#define MAIN_EXECUTION_PERIOD    100e-6     // main state machine pace period in [sec]
#define MAIN_EXEC_PER           (uint16_t)((CPU_FREQUENCY * MAIN_EXECUTION_PERIOD)-1)


    
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* APPLICATION_GLOBALS_HEADER_H */

