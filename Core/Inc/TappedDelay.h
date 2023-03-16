/*
 * TappedDelay.h
 *
 *  Created on: Nov 22, 2022
 *      Author: Fuguru
 */

#ifndef INC_TAPPEDDELAY_H_
#define INC_TAPPEDDELAY_H_

#include "Delay.h"
#include "stdint.h"

typedef struct {
	fDelay_TypeDef delay1;
	fDelay_TypeDef delay2;
	fDelay_TypeDef delay3;
	float g0, g1, g2, g3;
} fTappedDelay_TypeDef;

void TappedDelay_Init(fTappedDelay_TypeDef* hdelay, uint32_t d1, uint32_t d2, uint32_t d3);

float TappedDelay_Step(fTappedDelay_TypeDef* hdelay, float input);



#endif /* INC_TAPPEDDELAY_H_ */
