/*
 * Delay.h
 *
 *  Created on: Nov 14, 2022
 *      Author: Fuguru
 *
 *      https://ccrma.stanford.edu/realsimple/Delay/Delay_Line_C.html
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include "stdint.h"
#define DELAY_LENGTH 25000

typedef struct {
	float buffer[DELAY_LENGTH];
	uint32_t max_length;
	uint32_t length;
	int32_t position;
	int32_t inc;
} fDelay_TypeDef;

void Delay_Init(fDelay_TypeDef* hdelay, int32_t length);
float Delay_Step(fDelay_TypeDef* hdelay, float input);
float Delay_Peek(fDelay_TypeDef* hdelay);
void Delay_AdjustLength(fDelay_TypeDef* hdelay, int32_t length);

#endif /* INC_DELAY_H_ */
