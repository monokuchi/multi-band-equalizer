/*
 * EMAFilter.c
 *
 *  Created on: Sep 20, 2024
 *      Author: john
 */



/*
 * INCLUDES
 */
#include "EMAFilter.h"




/*
 * FUNCTIONS
 */
void EMAFilter_Init(EMAFilter *filter, float alpha)
{
	filter->out = 0.0f;
	EMAFilter_Set_Coefficients(filter, alpha);
}

void EMAFilters_Init(EMAFilter *filters, size_t filters_size, float alpha)
{
	for (size_t i=0; i<filters_size; ++i)
	{
		EMAFilter_Init(&filters[i], alpha);
	}
}

void EMAFilter_Set_Coefficients(EMAFilter *filter, float alpha)
{
	if (alpha < 0.0f)
	{
		filter->alpha = 0.0f;
	}
	else if (alpha > 1.0f)
	{
		filter->alpha = 1.0f;
	}
	else
	{
		filter->alpha = alpha;
	}
}

float EMAFilter_Update(EMAFilter *filter, float input_sample)
{
	/*
	 * Compute the output sample
	 */
	filter->out = (filter->alpha * input_sample) + ((1.0f-filter->alpha) * filter->out);
	return filter->out;
}
