/*
 * PeakingFilter.h
 *
 *  Created on: Jul 28, 2024
 *      Author: john
 */

#ifndef INC_PEAKINGFILTER_H_
#define INC_PEAKINGFILTER_H_



/*
 * INCLUDES
 */
#include <stdint.h>

#include "math.h"



/*
 * DEFINES
 */
#define PEAKING_FILT_PARAMS_ALL_PASS { .center_freq_hz = 1.0f, .bandwidth_hz = 0.000001f, .gain_linear = 1.0f };



/*
 * PeakingFilter STRUCT
 */
typedef struct
{
	/*
	 * Sample period in seconds
	 */
	float sample_period_sec;

	/*
	 * Input and output arrays
	 */
	float y[3];
	float x[3];

	/*
	 * Filter coefficients
	 * a -> y[n] coefficients
	 * b -> x[n] coefficients
	 */
	float a[3];
	float b[3];
} PeakingFilter;


/*
 * PeakingFilterParameters STRUCT
 */
typedef struct
{
	float center_freq_hz;
	float bandwidth_hz;
	float gain_linear; // Gain=1 -> All Pass, Gain>1 -> Boost, 0<Gain<1 -> Cut
} PeakingFilterParameters;





/*
 * FUNCTIONS
 */
void PeakingFilter_Init(PeakingFilter *filter, float sample_rate_hz);

void PeakingFilters_Init(PeakingFilter *filters, size_t filters_size, float sample_rate_hz);

void PeakingFilter_Set_Coefficients(PeakingFilter *filter, PeakingFilterParameters *filter_params);

float PeakingFilter_Update(PeakingFilter *filter, float input_sample);

float PeakingFilter_Update_Cascade(PeakingFilter *filters, size_t filters_size, float input_sample);




#endif /* INC_PEAKINGFILTER_H_ */
