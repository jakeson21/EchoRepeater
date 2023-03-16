/*
 * arm_IIR_AllpassPhaser.c
 *
 *  Created on: Nov 26, 2022
 *      Author: Fuguru
 */

#include "arm_IIR_AllpassPhaser.h"

void ArmAllpassPhaser_Update(ArmAllpassPhaser_t* hF)
{
	float32_t w0 = arm_sin_f32(arm_sin_f32(hF->pos) * M_PI)*hF->width*0.5 + hF->w_offset;
	float32_t a2 = hF->depth * hF->depth;

	//	b[0] = a2;
	//	b[1] = a1;
	//	b[2] = 1.0;
	//
	//	a[0] = 1.0;
	//	a[1] = a1;
	//	a[2] = a2;

	for (size_t n=0; n<NUM_SECTIONS; n++)
	{
		float32_t a1 = -2.0 * hF->depth * arm_cos_f32(w0 + 1.85*n*w0);

		size_t k = n*5;
		hF->coefficients[0 + k] = a2;
		hF->coefficients[1 + k] = a1;
		hF->coefficients[2 + k] = 1.0;
		hF->coefficients[3 + k] = -a1;
		hF->coefficients[4 + k] = -a2;
	}

	hF->F.pCoeffs = hF->coefficients;
}

void ArmAllpassPhaser_Init(ArmAllpassPhaser_t* hF,
	float depth,
	float period_s,
	float width_Hz,
	float sample_rate_Hz,
	size_t blockSize)
{
	hF->width = width_Hz / sample_rate_Hz;
	hF->w_start = 110.0 / sample_rate_Hz;
	hF->w_stop = hF->w_start + hF->width;
	hF->w_step = (2.0 / period_s) * (blockSize / sample_rate_Hz);
	hF->w_offset = hF->w_start + hF->width*0.5;
	hF->pos = 0; // ranges from [0, 2)
	hF->depth = depth;
	arm_biquad_cascade_df1_init_f32(&hF->F, NUM_SECTIONS, hF->coefficients, hF->taps);
	ArmAllpassPhaser_Update(hF);
}


void ArmAllpassPhaser_StepBlock(ArmAllpassPhaser_t* hF, float32_t* input, float32_t* dry_buffer, float32_t* output, size_t blockSize)
{
    arm_scale_f32(input, 0.495, input, blockSize);		// range is now from [-0.25, 0.25]
    arm_copy_f32(input, dry_buffer, blockSize);
	arm_biquad_cascade_df1_f32(&hF->F, input, output, blockSize);
	arm_add_f32(output, dry_buffer, output, blockSize);

	hF->pos += hF->w_step;
	if (hF->pos > 2.0)
	{
		hF->pos -= 2.0;
	}
	ArmAllpassPhaser_Update(hF);
}
