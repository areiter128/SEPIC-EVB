/* **********************************************************************************
 * z-Domain Control Loop Designer, Version 0.9.0.77
 * **********************************************************************************
 * 2p2z compensation filter coefficients derived for following operating
 * conditions:
 * **********************************************************************************
 *
 *  Controller Type:    2P2Z - Basic Current Mode Compensator
 *  Sampling Frequency: 350000 Hz
 *  Fixed Point Format: 15
 *  Scaling Mode:       3 - Dual Bit-Shift Scaling
 *  Input Gain:         0.148
 *
 * *********************************************************************************
 * CGS Version:         1.0.0
 * CGS Date:            11/08/19
 * *********************************************************************************
 * User:                M91406
 * Date/Time:           12/10/2019 1:09:45 PM
 * ********************************************************************************/

#include "c2p2z_sepic.h"

/* *********************************************************************************
 * Data Arrays:
 * This source file declares the default parameters of the z-domain compensation
 * filter. The cNPNZ_t data structure contains two pointers to A- and B-
 * coefficient arrays and two pointers to control and error history arrays.
 *
 * For optimized data processing during DSP computations, these arrays must be
 * located in specific memory locations (X-space for coefficient arrays and
 * Y-space for control and error history arrays).
 *
 * The following declarations are used to define the array data contents, their
 * length and memory location. These declarations are made publicly accessible
 * through defines in source file c2p2z_sepic.c
 * ********************************************************************************/

volatile C2P2Z_SEPIC_CONTROL_LOOP_COEFFICIENTS_t __attribute__((space(xmemory), near)) c2p2z_sepic_coefficients; // A/B-Coefficients
volatile uint16_t c2p2z_sepic_ACoefficients_size = (sizeof(c2p2z_sepic_coefficients.ACoefficients)/sizeof(c2p2z_sepic_coefficients.ACoefficients[0])); // A-coefficient array size
volatile uint16_t c2p2z_sepic_BCoefficients_size = (sizeof(c2p2z_sepic_coefficients.BCoefficients)/sizeof(c2p2z_sepic_coefficients.BCoefficients[0])); // B-coefficient array size

volatile C2P2Z_SEPIC_CONTROL_LOOP_HISTORIES_t __attribute__((space(ymemory), far)) c2p2z_sepic_histories; // Control/Error Histories
volatile uint16_t c2p2z_sepic_ControlHistory_size = (sizeof(c2p2z_sepic_histories.ControlHistory)/sizeof(c2p2z_sepic_histories.ControlHistory[0])); // Control history array size
volatile uint16_t c2p2z_sepic_ErrorHistory_size = (sizeof(c2p2z_sepic_histories.ErrorHistory)/sizeof(c2p2z_sepic_histories.ErrorHistory[0])); // Error history array size

/* *********************************************************************************
 * 	Pole&Zero Placement:
 * *********************************************************************************
 *
 *    fP0:    800 Hz
 *    fP1:    17000 Hz
 *    fZ1:    1200 Hz
 *
 * *********************************************************************************
 * 	Filter Coefficients and Parameters:
 * ********************************************************************************/
volatile fractional c2p2z_sepic_ACoefficients [2] =
{
    0x6F0E, // Coefficient A1 will be multiplied with controller output u(n-1)
    0xD0F3  // Coefficient A2 will be multiplied with controller output u(n-2)
};

volatile fractional c2p2z_sepic_BCoefficients [3] =
{
    0x4D28, // Coefficient B0 will be multiplied with error input e(n-0)
    0x01A5, // Coefficient B1 will be multiplied with error input e(n-1)
    0xB47E  // Coefficient B2 will be multiplied with error input e(n-2)
};

// Coefficient normalization factors
volatile int16_t c2p2z_sepic_pre_scaler = 3;
volatile int16_t c2p2z_sepic_post_shift_A = -1;
volatile int16_t c2p2z_sepic_post_shift_B = 0;
volatile fractional c2p2z_sepic_post_scaler = 0x0000;

volatile cNPNZ16b_t c2p2z_sepic; // user-controller data object

/* ********************************************************************************/

/*!c2p2z_sepic_Init()
 * *********************************************************************************
 * Summary: Initializes controller coefficient arrays and normalization
 *
 * Parameters:
 *     - cNPNZ16b_t* controller
 *
 * Returns:
 *     - uint16_t:  0=failure, 1=success
 *
 * Description:
 * This function needs to be called from user code once to initialize coefficient
 * arrays and number normalization settings of the c2p2z_sepic controller
 * object.
 *
 * PLEASE NOTE:
 * This routine DOES NOT initialize the complete controller object.
 * User-defined settings such as pointers to the control reference, source and
 * target registers, output minima and maxima and further, design-dependent
 * settings, need to be specified in user code.
 * ********************************************************************************/
volatile uint16_t c2p2z_sepic_Init(volatile cNPNZ16b_t* controller)
{
    volatile uint16_t i = 0;

    // Initialize controller data structure at runtime with pre-defined default values
    controller->status.value = CONTROLLER_STATUS_CLEAR;  // clear all status flag bits (will turn off execution))
    
    controller->ptrACoefficients = &c2p2z_sepic_coefficients.ACoefficients[0]; // initialize pointer to A-coefficients array
    controller->ptrBCoefficients = &c2p2z_sepic_coefficients.BCoefficients[0]; // initialize pointer to B-coefficients array
    controller->ptrControlHistory = &c2p2z_sepic_histories.ControlHistory[0]; // initialize pointer to control history array
    controller->ptrErrorHistory = &c2p2z_sepic_histories.ErrorHistory[0]; // initialize pointer to error history array
    controller->normPostShiftA = c2p2z_sepic_post_shift_A; // initialize A-coefficients/single bit-shift scaler
    controller->normPostShiftB = c2p2z_sepic_post_shift_B; // initialize B-coefficients/dual/post scale factor bit-shift scaler
    controller->normPostScaler = c2p2z_sepic_post_scaler; // initialize control output value normalization scaling factor
    controller->normPreShift = c2p2z_sepic_pre_scaler; // initialize A-coefficients/single bit-shift scaler
    
    controller->ACoefficientsArraySize = c2p2z_sepic_ACoefficients_size; // initialize A-coefficients array size
    controller->BCoefficientsArraySize = c2p2z_sepic_BCoefficients_size; // initialize A-coefficients array size
    controller->ControlHistoryArraySize = c2p2z_sepic_ControlHistory_size; // initialize control history array size
    controller->ErrorHistoryArraySize = c2p2z_sepic_ErrorHistory_size; // initialize error history array size
    
    
    // Load default set of A-coefficients from user RAM into X-Space controller A-array
    for(i=0; i<controller->ACoefficientsArraySize; i++)
    {
        c2p2z_sepic_coefficients.ACoefficients[i] = c2p2z_sepic_ACoefficients[i];
    }

    // Load default set of B-coefficients from user RAM into X-Space controller B-array
    for(i=0; i<controller->BCoefficientsArraySize; i++)
    {
        c2p2z_sepic_coefficients.BCoefficients[i] = c2p2z_sepic_BCoefficients[i];
    }

    // Clear error and control histories of the 3P3Z controller
    c2p2z_sepic_Reset(&c2p2z_sepic);
    
    return(1);
}
 
