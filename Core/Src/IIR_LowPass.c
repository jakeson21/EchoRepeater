/*
 * IIR_LowPass.c
 *
 *  Created on: Nov 13, 2022
 *      Author: Fuguru
 */

#include "IIR_LowPass.h"
#include "math.h"
#include "float.h"

void IIR_LPF_Init(IIR_LPF* hlpf, float omega)
{
	omega = omega * M_PI;
	float alpha = (1.0 - sin(omega)) / cos(omega);
	float gain = (1.0 - alpha) / 2.0;

	if ( fabs(alpha)< FLT_EPSILON*10) alpha = 0;
	if ( fabs(gain)< FLT_EPSILON*10) gain = 0;

	hlpf->b[0] = gain;
	hlpf->b[1] = gain;
	hlpf->b[2] = 0;

	hlpf->a[0] = 1.0;
	hlpf->a[1] = -alpha;
	hlpf->a[2] = 0;
}

float IIR_LPF_Step(IIR_LPF* hlpf, float input)
{
	// shift memory to left
	hlpf->x[0] = hlpf->x[1];
	hlpf->x[1] = hlpf->x[2];
	hlpf->y[0] = hlpf->y[1];
	hlpf->y[1] = hlpf->y[2];

	// compute next state
	hlpf->x[2] = input;
	hlpf->y[2] = (hlpf->b[0]*hlpf->x[2])
				+ (hlpf->b[1]*hlpf->x[1])
				+ (hlpf->b[2]*hlpf->x[0])
				- (hlpf->a[1]*hlpf->y[1])
				- (hlpf->a[2]*hlpf->y[0]);

	// get output
	return hlpf->y[2];
}

void IIR_LPF_Compute(IIR_LPF* hlpf, float* input, float* output, int32_t length)
{
	for (size_t m=0; m<length; m++)
	{
		output[m] = IIR_LPF_Step(hlpf, input[m]);
	}
}
