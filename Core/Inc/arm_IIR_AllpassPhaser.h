/*
 * arm_IIR_AllpassPhaser.h
 *
 *  Created on: Nov 26, 2022
 *      Author: Fuguru
 *
 *      https://github.com/ARM-software/CMSIS/blob/master/CMSIS/DSP_Lib/Source/FilteringFunctions/arm_biquad_cascade_df1_init_f32.c
 */

#ifndef INC_ARM_IIR_ALLPASSPHASER_H_
#define INC_ARM_IIR_ALLPASSPHASER_H_

#include "arm_math.h"

#define NUM_SECTIONS 4

typedef struct {
arm_biquad_casd_df1_inst_f32 F;
float32_t taps[4*NUM_SECTIONS];
float32_t coefficients[5*NUM_SECTIONS];

float32_t w_start;
float32_t w_stop;
float32_t w_mid;
float32_t w_offset;
float32_t w_step;
float32_t width;
float32_t depth;
float32_t pos;
} ArmAllpassPhaser_t;

void ArmAllpassPhaser_Update(ArmAllpassPhaser_t* hF);
void ArmAllpassPhaser_Init(ArmAllpassPhaser_t* hF, float depth, float period_s, float width_Hz, float sample_rate_Hz, size_t blockSize);
void ArmAllpassPhaser_StepBlock(ArmAllpassPhaser_t* hF, float32_t* input, float32_t* dry, float32_t* output, size_t blockSize);


#endif /* INC_ARM_IIR_ALLPASSPHASER_H_ */
