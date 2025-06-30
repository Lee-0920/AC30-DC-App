/*
 * InfrSensor.c
 *
 *  Created on: 2024年6月19日
 *      Author: admin
 */

#include <LiquidDriver/RelayDriver.h>

void Realy_Init(RelayDriver *realy)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(realy->rcc, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Pin = realy->pin;
	GPIO_Init(realy->port, &GPIO_InitStructure);
}

void Relay_On(RelayDriver *realy)
{
	GPIO_SetBits(realy->port, realy->pin);
}

void Relay_Off(RelayDriver *realy)
{
	GPIO_ResetBits(realy->port, realy->pin);
}

RelayStatus Relay_Read(RelayDriver *realy)
{
	 if(GPIO_ReadOutputDataBit(realy->port, realy->pin))
	{
		return RELAY_BUSY;
	}
	else
	{
		return RELAY_IDLE;
	}
}
