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
	filter->a[0] = 4.0f + (2.0f * (filter_params->gain_linear / Q) * wc_T) + (wc_T * wc_T);
	filter->a[1] = (2.0f * (wc_T * wc_T)) - 8.0f;
	filter->a[2] = 4.0f - (2.0f * (filter_params->gain_linear / Q) * wc_T) + (wc_T * wc_T);

	filter->b[0] = 1.0f / (4.0f + ((2.0f / filter_params->gain_linear) * wc_T) + (wc_T * wc_T)); // NOTE: This is the reciprocal of the coefficient (saves one extra division in the update step)
	filter->b[1] = -((2.0f * (wc_T * wc_T)) - 8.0f);
	filter->b[2] = -(4.0f - ((2.0f / filter_params->gain_linear) * wc_T) + (wc_T * wc_T));
}

float PeakingFilter_Update(PeakingFilter *filter, float input_sample)
{
	/*
	 * Compute the output sample
	 */
	float output_sample = filter->b[0] * ((filter->a[0]*input_sample) + (filter->a[1]*filter->x[1]) + (filter->a[2]*filter->x[2]) +
									      (filter->b[1]*filter->y[1]) + (filter->b[2]*filter->y[2]));


	/*
	 * Update the filter inputs and outputs
	 */
	filter->x[0] = input_sample;
	filter->x[1] = filter->x[0];
	filter->x[2] = filter->x[1];

	filter->y[0] = output_sample;
	filter->y[1] = filter->y[0];
	filter->y[2] = filter->y[1];

	return output_sample;
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
