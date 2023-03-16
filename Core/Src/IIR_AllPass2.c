/*
 * IIR_HighPass.c
 *
 *  Created on: Nov 17, 2022
 *      Author: Fuguru
 *
 *      https://community.st.com/s/article/configuring-dsp-libraries-on-stm32cubeide
 *      https://arm-software.github.io/CMSIS_5/latest/DSP/html/index.html
 *
 */

#include "IIR_AllPass2.h"
#include "math.h"
#include "arm_math.h"
#include "float.h"


#define MIN_VAL FLT_EPSILON*10

void IIR_AP2_Init(IIR_AP2* hAPF2, float omega, float R)
{
	hAPF2->r = R;
	hAPF2->w0 = omega;

	omega = omega * M_PI;
	float a1 = -2*R*arm_cos_f32(omega); //cos(omega);
	float a2 = R*R;

	if ( fabs(a1)< MIN_VAL) a1 = 0;
	if ( fabs(a2)< MIN_VAL) a2 = 0;

	hAPF2->b[0] = a2;
	hAPF2->b[1] = a1;
	hAPF2->b[2] = 1.0;

	hAPF2->a[0] = 1.0;
	hAPF2->a[1] = a1;
	hAPF2->a[2] = a2;
}

float IIR_AP2_Step(IIR_AP2* hAPF2, float input)
{
	// shift memory to left
	hAPF2->x[0] = hAPF2->x[1];
	hAPF2->x[1] = hAPF2->x[2];
	hAPF2->y[0] = hAPF2->y[1];
	hAPF2->y[1] = hAPF2->y[2];

	// compute next state
	hAPF2->x[2] = input;
	hAPF2->y[2] = hAPF2->b[0]*hAPF2->x[2] + hAPF2->b[1]*hAPF2->x[1] + hAPF2->b[2]*hAPF2->x[0] - hAPF2->a[1]*hAPF2->y[1] - hAPF2->a[2]*hAPF2->y[0];

	// get output
	return hAPF2->y[2];
}

void IIR_AP2_Compute(IIR_AP2* hAPF2, float* input, float* output, int32_t length)
{
	for (size_t m=0; m<length; m++)
	{
		output[m] = IIR_AP2_Step(hAPF2, input[m]);
	}
}
