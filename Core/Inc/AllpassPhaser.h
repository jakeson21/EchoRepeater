/*
 * AllpassPhaser.h
 *
 *  Created on: Nov 25, 2022
 *      Author: Fuguru
 */

#ifndef INC_ALLPASSPHASER_H_
#define INC_ALLPASSPHASER_H_

#include "IIR_AllPass2.h"
#include "arm_math.h"

#define NUMSECTIONS 4

typedef struct {
	IIR_AP2 ap[NUMSECTIONS];
	float w0[NUMSECTIONS];
	float R[NUMSECTIONS];
	float gain;
	float d_w0;
	int32_t N;

	arm_biquad_casd_df1_inst_f32 F;
	float32_t taps[4 * NUMSECTIONS];
	float32_t coefficients[5 * NUMSECTIONS];

} AllpassPhaser_t;

void AllpassPhaser_Init(AllpassPhaser_t* hfilter, float rate, float depth);
float AllpassPhaser_Step(AllpassPhaser_t* hfilter, float input);
void AllpassPhaser_StepBlock(AllpassPhaser_t* hfilter, float* input, float* output, size_t length);


#endif /* INC_ALLPASSPHASER_H_ */
