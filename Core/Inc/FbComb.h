/*
 * FbComb.h
 *
 *  Created on: Nov 23, 2022
 *      Author: Fuguru
 *
 *      https://ccrma.stanford.edu/realsimple/Delay/Delay_Line_C.html
 */

#ifndef INC_FBCOMB_H_
#define INC_FBCOMB_H_


#include "Delay.h"
#include "stdint.h"

typedef struct {
	fDelay_TypeDef delay1;
	float a, b0;
} fFbComb_TypeDef;

void FbComb_Init(fFbComb_TypeDef* hfilter, uint32_t length, float b0, float a);

float FbComb_Step(fFbComb_TypeDef* hfilter, float input);


#endif /* INC_FBCOMB_H_ */
