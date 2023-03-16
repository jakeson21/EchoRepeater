/*
 * IIR_Peak.c
 *
 *  Created on: Nov 17, 2022
 *      Author: Fuguru
 */

#include "IIR_Peak.h"
#include "math.h"
#include "float.h"

void IIR_Peak_Init(IIR_Peak* hPeak, float omega, float bandwidth)
{
	omega = omega * M_PI;
	bandwidth = bandwidth * M_PI;
	float alpha = (1.0 - sin(bandwidth)) / cos(bandwidth);
	float beta = cos(omega);
	float gain = (1.0 - alpha) / 2.0;

	if ( fabs(alpha)< FLT_EPSILON*10) alpha = 0;
	if ( fabs(beta)< FLT_EPSILON*10) beta = 0;
	if ( fabs(gain)< FLT_EPSILON*10) gain = 0;

	hPeak->b[0] = gain;
	hPeak->b[1] = 0;
	hPeak->b[2] = -gain;

	hPeak->a[0] = 1.0;
	hPeak->a[1] = -beta*(1.0 + alpha);
	hPeak->a[2] = alpha;
}

float IIR_Peak_Step(IIR_Peak* hPeak, float input)
{
	// shift memory to left
	hPeak->x[0] = hPeak->x[1];
	hPeak->x[1] = hPeak->x[2];
	hPeak->y[0] = hPeak->y[1];
	hPeak->y[1] = hPeak->y[2];

	// compute next state
	hPeak->x[2] = input;
	hPeak->y[2] = hPeak->b[0]*hPeak->x[2] + hPeak->b[1]*hPeak->x[1] + hPeak->b[2]*hPeak->x[0] - hPeak->a[1]*hPeak->y[1] - hPeak->a[2]*hPeak->y[0];

	// get output
	return hPeak->y[2];
}

void IIR_Peak_Compute(IIR_Peak* hPeak, float* input, float* output, int32_t length)
{
	for (size_t m=0; m<length; m++)
	{
		output[m] = IIR_Peak_Step(hPeak, input[m]);
	}
}
