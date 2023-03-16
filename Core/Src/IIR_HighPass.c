/*
 * IIR_HighPass.c
 *
 *  Created on: Nov 17, 2022
 *      Author: Fuguru
 */

#include "IIR_HighPass.h"
#include "math.h"
#include "float.h"

void IIR_HPF_Init(IIR_HPF* hHPF, float omega)
{
	omega = omega * M_PI;
	float alpha = (1.0 - sin(omega)) / cos(omega);
	float gain = (1.0 + alpha) / 2.0;

	if ( fabs(alpha)< FLT_EPSILON*10) alpha = 0;
	if ( fabs(gain)< FLT_EPSILON*10) gain = 0;

	hHPF->b[0] = gain;
	hHPF->b[1] = -gain;
	hHPF->b[2] = 0;

	hHPF->a[0] = 1.0;
	hHPF->a[1] = -alpha;
	hHPF->a[2] = 0;
}

float IIR_HPF_Step(IIR_HPF* hHPF, float input)
{
	// shift memory to left
	hHPF->x[0] = hHPF->x[1];
	hHPF->x[1] = hHPF->x[2];
	hHPF->y[0] = hHPF->y[1];
	hHPF->y[1] = hHPF->y[2];

	// compute next state
	hHPF->x[2] = input;
	hHPF->y[2] = hHPF->b[0]*hHPF->x[2] + hHPF->b[1]*hHPF->x[1] + hHPF->b[2]*hHPF->x[0] - hHPF->a[1]*hHPF->y[1] - hHPF->a[2]*hHPF->y[0];

	// get output
	return hHPF->y[2];
}

void IIR_HPF_Compute(IIR_HPF* hHPF, float* input, float* output, int32_t length)
{
	for (size_t m=0; m<length; m++)
	{
		output[m] = IIR_HPF_Step(hHPF, input[m]);
	}
}
