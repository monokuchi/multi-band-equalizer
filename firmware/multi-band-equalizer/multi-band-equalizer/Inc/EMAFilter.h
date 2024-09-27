/*
 * EMAFilter.h
 *
 *  Created on: Sep 20, 2024
 *      Author: john
 */

#ifndef INC_EMAFILTER_H_
#define INC_EMAFILTER_H_




/*
 * INCLUDES
 */
#include <stddef.h>


/*
 * DEFINES
 */


/*
 * EMAFilter STRUCT
 */
typedef struct
{
	/*
	 * Variable to store filter output
	 */
	float out;

	/*
	 * Filter coefficients
	 * alpha -> [0.0, 1.0]
	 */
	float alpha;
} EMAFilter;




/*
 * FUNCTIONS
 */
void EMAFilter_Init(EMAFilter *filter, float alpha);

void EMAFilters_Init(EMAFilter *filters, size_t filters_size, float alpha);

void EMAFilter_Set_Coefficients(EMAFilter *filter, float alpha);

float EMAFilter_Update(EMAFilter *filter, float input_sample);



#endif /* INC_EMAFILTER_H_ */
