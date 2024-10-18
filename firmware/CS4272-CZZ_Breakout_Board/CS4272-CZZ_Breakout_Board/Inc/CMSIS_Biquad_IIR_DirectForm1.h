/*
 * CMSIS_Biquad_IIR_DirectForm1.h
 *
 *  Created on: Oct 5, 2024
 *      Author: john
 */

#ifndef INC_CMSIS_BIQUAD_IIR_DIRECTFORM1_H_
#define INC_CMSIS_BIQUAD_IIR_DIRECTFORM1_H_



/*
 * INCLUDES
 */
#include "main.h" // Needed for __FPU_PRESENT

#ifndef ARM_MATH_CM7
	#define ARM_MATH_CM7
#endif
#include "arm_math.h"





/*
 * DEFINES
 */
#define IIR_DIRECT_FORM_1_NUM_STATES 4
#define IIR_DIRECT_FORM_1_NUM_COEFFICIENTS 5
#define IIR_DIRECT_FORM_1_NUM_STAGES 7 // Number of 2nd order stages in the IIR filter




/*
 * IIR_Direct_Form_1 STRUCT
 */
typedef struct
{
	/*
	 * Instance of ARM's CMSIS IIR struct
	 */
	arm_biquad_casd_df1_inst_f32 iir_filter;

	/*
	 * Stores the filter's state, must be in this format ({x[n-1], x[n-2], y[n-1], y[n-2]})
	 * Each stage contributes 4 state variables
	 */
	float cmsis_state[IIR_DIRECT_FORM_1_NUM_STATES*IIR_DIRECT_FORM_1_NUM_STAGES];

	/*
	 * Stores the filter coefficients (a->Input, b->Output)
	 * Must be in this format ({b10, b11, b12, a11, a12, b20, b21, b22, a21, a22, ...})
	 * Each stage contributes 5 coefficients
	 */
	float cmsis_coefficients[IIR_DIRECT_FORM_1_NUM_COEFFICIENTS*IIR_DIRECT_FORM_1_NUM_STAGES];

	/*
	 * Sample period in seconds
	 */
	float sample_period_sec;
} IIR_Direct_Form_1;



/*
 * FUNCTIONS
 */
void IIR_Direct_Form_1_Init(IIR_Direct_Form_1 *filter);

void IIR_Direct_Form_1_Set_Coefficients(IIR_Direct_Form_1 *filter);

void IIR_Direct_Form_1_Update(IIR_Direct_Form_1 *filter, float *src, float *dst, size_t block_size);

IIR_Direct_Form_1 IIR_Direct_Form_1_Cascade_Filters(const IIR_Direct_Form_1 *filter_1, const IIR_Direct_Form_1 *filter_2);





#endif /* INC_CMSIS_BIQUAD_IIR_DIRECTFORM1_H_ */
