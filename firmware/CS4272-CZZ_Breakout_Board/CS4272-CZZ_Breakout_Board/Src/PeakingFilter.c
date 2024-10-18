/*
 * PeakingFilter.c
 *
 *  Created on: Jul 28, 2024
 *      Author: john
 */


/*
 * INCLUDES
 */
#include "PeakingFilter.h"




/*
 * FUNCTIONS
 */
void PeakingFilter_Init(PeakingFilter *filter, float sample_rate_hz)
{
	filter->sample_period_sec = 1.0f / sample_rate_hz;

	// Zero out the input and output arrays
	for (size_t i=0; i<3; ++i)
	{
		filter->y[i] = 0.0f;
		filter->x[i] = 0.0f;
	}

	// Compute the default filter coefficients (all-pass filter)
	PeakingFilterParameters default_filter_params = PEAKING_FILT_PARAMS_ALL_PASS;
	PeakingFilter_Set_Coefficients(filter, &default_filter_params);
}

void PeakingFilters_Init(PeakingFilter *filters, size_t filters_size, float sample_rate_hz)
{
	for (size_t i=0; i<filters_size; ++i)
	{
		PeakingFilter_Init(&filters[i], sample_rate_hz);
	}
}

void PeakingFilter_Set_Coefficients(PeakingFilter *filter, PeakingFilterParameters *filter_params)
{
	// Compute the pre-warp cut off frequency * T -> w_cd * T = (2/T) * tan(w_c * T/2) * T = 2*tan(pi * f_c * T)
	float wc_T = 2.0f * tanf(M_PI * filter_params->center_freq_hz * filter->sample_period_sec);

	// Compute the Q factor
	float Q = filter_params->center_freq_hz / filter_params->bandwidth_hz;


	/*
	 * Compute our filter coefficients
	 */
	filter->a[0] = 1.0f / (4.0f + ((2.0f / Q) * wc_T) + (wc_T * wc_T)); // NOTE: This is the reciprocal of the coefficient (saves one extra division in the update step)
	filter->a[1] = (2.0f * (wc_T * wc_T)) - 8.0f;
	filter->a[2] = 4.0f - ((2.0f / Q) * wc_T) + (wc_T * wc_T);

	filter->b[0] = 4.0f + (2.0f * (filter_params->gain_linear / Q) * wc_T) + (wc_T * wc_T);
	filter->b[1] = (2.0f * (wc_T * wc_T)) - 8.0f;
	filter->b[2] = 4.0f - (2.0f * (filter_params->gain_linear / Q) * wc_T) + (wc_T * wc_T);
}

float PeakingFilter_Update(PeakingFilter *filter, float input_sample)
{
	/*
	 * Shift the filter inputs and outputs
	 */
	filter->x[2] = filter->x[1];
	filter->x[1] = filter->x[0];
	filter->x[0] = input_sample;

	filter->y[2] = filter->y[1];
	filter->y[1] = filter->y[0];


	/*
	 * Compute the output sample
	 */
	filter->y[0] = filter->a[0] * ((filter->b[0]*filter->x[0]) + (filter->b[1]*filter->x[1]) + (filter->b[2]*filter->x[2])
								  -(filter->a[1]*filter->y[1]) - (filter->a[2]*filter->y[2]));

	return filter->y[0];
}

float PeakingFilter_Update_Cascade(PeakingFilter *filters, size_t filters_size, float input_sample)
{
	/*
	 * Cascades the peaking filters provided by "filters" and runs our input sample through them
	 * As we go away from f_c we get unity gain, this property of peaking filters allows us to cascade them properly
	 */
	float output_sample = input_sample;
	for (size_t i=0; i<filters_size; ++i)
	{
		output_sample = PeakingFilter_Update(&filters[i], output_sample);
	}
	return output_sample;
}












void PeakingFilter_Init_CMSIS(IIR_Direct_Form_1 *iir_filter, float sample_rate_hz)
{
	iir_filter->sample_period_sec = 1.0f / sample_rate_hz;

	// Set default filter coefficients (all-pass filter)
	PeakingFilterParameters default_filter_params[IIR_DIRECT_FORM_1_NUM_STAGES] = { [0 ... IIR_DIRECT_FORM_1_NUM_STAGES-1] = PEAKING_FILT_PARAMS_ALL_PASS };
	PeakingFilter_Set_Coefficients_CMSIS(iir_filter, default_filter_params, IIR_DIRECT_FORM_1_NUM_STAGES);

	IIR_Direct_Form_1_Init(iir_filter);
}

void PeakingFilter_Set_Coefficients_CMSIS(IIR_Direct_Form_1 *iir_filter, PeakingFilterParameters *filter_params, size_t filters_size)
{
	// Compute filter coefficients and store them in a format that the CMSIS init function expects ({b10, b11, b12, a11, a12, b20, b21, b22, a21, a22, ...})
	for (size_t i=0; i<filters_size; ++i)
	{
		// Compute the pre-warp cut off frequency * T -> w_cd * T = (2/T) * tan(w_c * T/2) * T = 2*tan(pi * f_c * T)
		float wc_T = 2.0f * tanf(M_PI * filter_params[i].center_freq_hz * iir_filter->sample_period_sec);

		// Compute the Q factor
		float Q = filter_params[i].center_freq_hz / filter_params[i].bandwidth_hz;


		/*
		 * Compute our filter coefficients
		 */
		float a_0 = 1.0f / (4.0f + ((2.0f / Q) * wc_T) + (wc_T * wc_T)); // NOTE: This is the reciprocal of the coefficient (saves one extra division in the normalization step)
		float a_1 = (2.0f * (wc_T * wc_T)) - 8.0f;
		float a_2 = 4.0f - ((2.0f / Q) * wc_T) + (wc_T * wc_T);

		float b_0 = 4.0f + (2.0f * (filter_params[i].gain_linear / Q) * wc_T) + (wc_T * wc_T);
		float b_1 = (2.0f * (wc_T * wc_T)) - 8.0f;
		float b_2 = 4.0f - (2.0f * (filter_params[i].gain_linear / Q) * wc_T) + (wc_T * wc_T);

		// Normalize the coefficients by a_0
		a_1 *= a_0;
		a_2 *= a_0;

		b_0 *= a_0;
		b_1 *= a_0;
		b_2 *= a_0;


		/*
		 * Store our filter coefficients
		 */
		iir_filter->cmsis_coefficients[(IIR_DIRECT_FORM_1_NUM_COEFFICIENTS*i)] = b_0;
		iir_filter->cmsis_coefficients[(IIR_DIRECT_FORM_1_NUM_COEFFICIENTS*i)+1] = b_1;
		iir_filter->cmsis_coefficients[(IIR_DIRECT_FORM_1_NUM_COEFFICIENTS*i)+2] = b_2;

		// CMSIS DSP Library uses positive signs on their y coefficients, so negate to compensate
		iir_filter->cmsis_coefficients[(IIR_DIRECT_FORM_1_NUM_COEFFICIENTS*i)+3] = -a_1;
		iir_filter->cmsis_coefficients[(IIR_DIRECT_FORM_1_NUM_COEFFICIENTS*i)+4] = -a_2;
	}

	// Update our CMSIS IIR filter with our new cmsis_coefficients
	IIR_Direct_Form_1_Set_Coefficients(iir_filter);
}


void PeakingFilter_Update_CMSIS(IIR_Direct_Form_1 *iir_filter, float *input_samples, float *output_samples, size_t samples_block_size)
{
	IIR_Direct_Form_1_Update(iir_filter, input_samples, output_samples, samples_block_size);
}


