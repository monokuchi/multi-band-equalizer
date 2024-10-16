/*
 * CMSIS_Biquad_IIR_DirectForm1.c
 *
 *  Created on: Oct 5, 2024
 *      Author: john
 */




/*
 * INCLUDES
 */
#include "CMSIS_Biquad_IIR_DirectForm1.h"



/*
 * FUNCTIONS
 */
void IIR_Direct_Form_1_Init(IIR_Direct_Form_1 *filter)
{
	arm_biquad_cascade_df1_init_f32(&(filter->iir_filter), (uint8_t) IIR_DIRECT_FORM_1_NUM_STAGES, (float32_t *) (filter->cmsis_coefficients), (float32_t *) (filter->cmsis_state));
}

void IIR_Direct_Form_1_Set_Coefficients(IIR_Direct_Form_1 *filter)
{
	// Assumes that filter->cmsis_coefficients are initialized with values and arm_biquad_cascade_df1_init_f32() was previously called on the filter
	for (size_t i=0; i<IIR_DIRECT_FORM_1_NUM_COEFFICIENTS*IIR_DIRECT_FORM_1_NUM_STAGES; ++i)
	{
		filter->iir_filter.pCoeffs[i] = filter->cmsis_coefficients[i];
	}
}

void IIR_Direct_Form_1_Update(IIR_Direct_Form_1 *filter, float *src, float *dst, size_t block_size)
{
	arm_biquad_cascade_df1_f32(&(filter->iir_filter), (float32_t *) src, (float32_t *) dst, (uint32_t) block_size);
}

IIR_Direct_Form_1 IIR_Direct_Form_1_Cascade_Filters(const IIR_Direct_Form_1 *filter_1, const IIR_Direct_Form_1 *filter_2)
{
	IIR_Direct_Form_1 cascaded_filter;



	cascaded_filter.iir_filter.numStages = filter_1->iir_filter.numStages + filter_1->iir_filter.numStages;


	cascaded_filter.sample_period_sec = filter_1->sample_period_sec;

	return cascaded_filter;
}
