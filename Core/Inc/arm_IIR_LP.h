/*
 * arm_IIR_LP.h
 *
 *  Created on: Nov 26, 2022
 *      Author: Fuguru
 *
 *      https://github.com/ARM-software/CMSIS/blob/master/CMSIS/DSP_Lib/Source/FilteringFunctions/arm_biquad_cascade_df1_init_f32.c
 */

#ifndef INC_ARM_IIR_LP_H_
#define INC_ARM_IIR_LP_H_

#include "arm_math.h"

#define ARM_IIR_LP_NUM_SECTIONS 3

typedef struct {
	arm_biquad_casd_df1_inst_f32 F;
	float32_t taps[4*ARM_IIR_LP_NUM_SECTIONS];
	float32_t coefficients[5*ARM_IIR_LP_NUM_SECTIONS];
} ArmIIR_LPF_t;

void ArmIIR_LPF_Init(ArmIIR_LPF_t* hF);
void ArmIIR_LPF_StepBlock(ArmIIR_LPF_t* hF, float32_t* input, float32_t* output, size_t blockSize);


#endif /* INC_ARM_IIR_LP_H_ */
