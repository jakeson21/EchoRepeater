/*
 * arm_IIR_LP.c
 *
 *  Created on: Nov 26, 2022
 *      Author: Fuguru
 */

#include "arm_IIR_LP.h"

void ArmIIR_LPF_Init(ArmIIR_LPF_t* hF)
{
	// The coefficients are stored in the array <code>pCoeffs</code> in the following order:
	// {b10, b11, b12, a11, a12, b20, b21, b22, a21, a22, ...}, a1n is assumed to be 1.0

	float b[] = {0.24198265, 0.4839653,  0.24198265,
				 0.21425487, 0.42850975, 0.21425487,
				 0.14890378, 0.29780755, 0.14890378};
	float a[] = {-1.03206941,  0.27570794,
				 -1.1429805,   0.4128016,
				 -1.40438489,  0.73591519};

	for (size_t n=0; n<ARM_IIR_LP_NUM_SECTIONS; n++)
	{
		size_t k = n*5;
		hF->coefficients[0 + k] = b[0 + 3*n] / 3;
		hF->coefficients[1 + k] = b[1 + 3*n] / 3;
		hF->coefficients[2 + k] = b[2 + 3*n] / 3;
		hF->coefficients[3 + k] = -a[0 + 2*n];
		hF->coefficients[4 + k] = -a[1 + 2*n];
	}

	hF->F.pCoeffs = hF->coefficients;
	arm_biquad_cascade_df1_init_f32(&hF->F, ARM_IIR_LP_NUM_SECTIONS, hF->coefficients, hF->taps);
}


void ArmIIR_LPF_StepBlock(ArmIIR_LPF_t* hF, float32_t* input, float32_t* output, size_t blockSize)
{
	arm_biquad_cascade_df1_f32(&hF->F, input, output, blockSize);
}
