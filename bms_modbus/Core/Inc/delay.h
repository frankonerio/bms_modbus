/*
 * delay.h
 *
 *  Created on: Feb 12, 2024
 *      Author: frank
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include "stdint.h"
#include "main.h"

extern uint32_t Time_Cnt1;
extern uint32_t Time_Cnt2;



void Delayus(uint32_t usdelay);
void Delayms(uint32_t msdelay);
void DelayIncCnt(void);

#define DELAY_Time_1()			(Time_Cnt1)
#define DELAY_SetTime_1(time)		(Time_Cnt1 = (time))
#define DELAY_Time_2()			(Time_Cnt2)
#define DELAY_SetTime_2(time)		(Time_Cnt2 = (time))



#endif /* INC_DELAY_H_ */
