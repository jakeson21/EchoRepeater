/*
 * AllpassPhaser.c
 *
 *  Created on: Nov 25, 2022
 *      Author: Fuguru
 */

#include "AllpassPhaser.h"

void AllpassPhaser_Init(AllpassPhaser_t* hF, float rate, float depth)
{
	hF->w0[0] = 0.2;
	hF->w0[1] = hF->w0[0]*1.5;
	hF->w0[2] = hF->w0[1]*1.5;
	hF->w0[3] = hF->w0[2]*1.5;

	hF->R[0] = depth;
	hF->R[1] = depth;
	hF->R[2] = depth;
	hF->R[3] = depth;

	hF->d_w0 = rate / 25000.0;
	hF->N = 25000;
	hF->gain = 0.5;

	for (int n=0; n<NUMSECTIONS; n++)
	{
		IIR_AP2_Init(&hF->ap[n], hF->w0[n], hF->R[n]);
	}


	for (int n=0; n<NUMSECTIONS; n++)
	{
		hF->coefficients[n*5] = hF->ap[n].b[0] / 2;
		hF->coefficients[n*5+1] = hF->ap[n].b[1] / 2;
		hF->coefficients[n*5+2] = hF->ap[n].b[2] / 2;
		hF->coefficients[n*5+3] = hF->ap[n].a[1] / 2;
		hF->coefficients[n*5+4] = hF->ap[n].a[2] / 2;
	}
	arm_biquad_cascade_df1_init_f32(&hF->F, NUMSECTIONS, hF->coefficients, hF->taps);
}

float AllpassPhaser_Step(AllpassPhaser_t* hF, float input)
{
	float tmp = IIR_AP2_Step(&hF->ap[0], input);
	for (int n=1; n<NUMSECTIONS; n++)
	{
		tmp = IIR_AP2_Step(&hF->ap[n], tmp);
	}
	float y = tmp + input*hF->gain;

	static int32_t count = 0;
	if (++count >= hF->N)
	{
		hF->d_w0 = -hF->d_w0;
		count = 0;
	}

	hF->w0[0] += hF->d_w0;
	hF->w0[1] = hF->w0[0]*1.25;
	hF->w0[2] = hF->w0[1]*1.25;
	hF->w0[3] = hF->w0[2]*1.25;
	for (int n=0; n<4; n++)
	{
		IIR_AP2_Init(&hF->ap[n], hF->w0[n], hF->R[n]);
	}


	return y;
}


void AllpassPhaser_StepBlock(AllpassPhaser_t* hF, float* input, float* output, size_t length)
{
	arm_biquad_cascade_df1_f32(&hF->F, input, output, 1);
}

