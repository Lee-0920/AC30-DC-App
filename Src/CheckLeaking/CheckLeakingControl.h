/*
 * CheckLeakingControl.h
 *
 *  Created on: 2021年10月11日
 *      Author: liwenqin
 */

#ifndef SRC_CHECKLEAKING_CHECKLEAKINGCONTROL_H_
#define SRC_CHECKLEAKING_CHECKLEAKINGCONTROL_H_

#include "Common/Types.h"

#define MAX_CHANEL  	2

void CheckLeakingControl_Init(void);
Uint16 CheckLeakingControl_GetAD(Uint8 index);
void CheckLeaking_SetCheckLeakingReportPeriod(float period);


#endif /* SRC_CHECKLEAKING_CHECKLEAKINGCONTROL_H_ */
