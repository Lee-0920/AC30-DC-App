/*
 * InfrSensorMap.c
 *
 *  Created on: 2024年6月19日
 *      Author: admin
 */
#include <LiquidDriver/RelayMap.h>
#include <LiquidDriver/RelayDriver.h>
#include "Tracer/Trace.h"

RelayDriver s_relay[MAX_RELAY_NUM];

void RelayMap_Init(void)
{
	//接收引脚0
	s_relay[0].pin = GPIO_Pin_4;
	s_relay[0].port = GPIOE;
	s_relay[0].rcc = RCC_AHB1Periph_GPIOE;
	Realy_Init(&s_relay[0]);

	//接收引脚1
	s_relay[1].pin = GPIO_Pin_5;
	s_relay[1].port = GPIOE;
	s_relay[1].rcc = RCC_AHB1Periph_GPIOE;
	Realy_Init(&s_relay[1]);

	//接收引脚0
	s_relay[2].pin = GPIO_Pin_6;
	s_relay[2].port = GPIOE;
	s_relay[2].rcc = RCC_AHB1Periph_GPIOE;
	Realy_Init(&s_relay[2]);
}

Bool RelayMap_On(Uint8 index)
{
	if(index < MAX_RELAY_NUM)
	{
		Relay_On(&s_relay[index]);
		TRACE_INFO("\n Turn On [%d] Relay", index);
		return TRUE;
	}
	else
	{
		TRACE_ERROR("\n Invalid Index");
		return FALSE;
	}
}

Bool RelayMap_Off(Uint8 index)
{
	if(index < MAX_RELAY_NUM)
	{
		Relay_Off(&s_relay[index]);
		TRACE_INFO("\n Turn Off [%d] Relay", index);
		return TRUE;
	}
	else
	{
		TRACE_ERROR("\n Invalid Index");
		return FALSE;
	}
}

RelayStatus RelayMap_Read(Uint8 index)
{
	if(index < MAX_RELAY_NUM)
	{
		return Relay_Read(&s_relay[index]);
	}
	else
	{
		TRACE_ERROR("\n Invalid Index");
		return MAX_STATUS;
	}
}
