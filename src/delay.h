/*
 * delay.h
 *
 *  Created on: 31 Oct 2016
 *      Author: Steve Chang
 */

#ifndef __DELAY_H
#define __DELAY_H 			   
#include "stm32f10x.h"

void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);
void delay(u32 x);

#endif
