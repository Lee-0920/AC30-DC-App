/*
 * ValveMap.c
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */

#include <LiquidDriver/ValveMap.h>
#include "stm32f4xx.h"
#include "SolenoidValve/ValveManager.h"

//static Valve s_valveDir[2];

void ValveMap_Init(Valve *valve)
{
    Uint8 i;

    //球阀↓↓↓↓↓
    valve[0].pin = GPIO_Pin_0;
	valve[0].port= GPIOB;
	valve[0].rcc = RCC_AHB1Periph_GPIOB;
	ValveDriver_Init(&valve[0]);

	valve[1].pin = GPIO_Pin_1;
	valve[1].port= GPIOB;
	valve[1].rcc = RCC_AHB1Periph_GPIOB;
	ValveDriver_Init(&valve[1]);

    valve[2].pin = GPIO_Pin_10;
    valve[2].port = GPIOE;
    valve[2].rcc = RCC_AHB1Periph_GPIOE;
    ValveDriver_Init(&valve[2]);

    valve[3].pin = GPIO_Pin_11;
    valve[3].port = GPIOE;
    valve[3].rcc = RCC_AHB1Periph_GPIOE;
    ValveDriver_Init(&valve[3]);

    valve[4].pin = GPIO_Pin_12;
    valve[4].port = GPIOE;
    valve[4].rcc = RCC_AHB1Periph_GPIOE;
    ValveDriver_Init(&valve[4]);

    valve[5].pin = GPIO_Pin_13;
    valve[5].port= GPIOE;
    valve[5].rcc =  RCC_AHB1Periph_GPIOE;
    ValveDriver_Init(&valve[5]);

    valve[6].pin = GPIO_Pin_14;
    valve[6].port = GPIOE;
    valve[6].rcc = RCC_AHB1Periph_GPIOE;
    ValveDriver_Init(&valve[6]);

    valve[7].pin = GPIO_Pin_15;
    valve[7].port = GPIOE;
    valve[7].rcc = RCC_AHB1Periph_GPIOE;
    ValveDriver_Init(&valve[7]);

    valve[8].pin = GPIO_Pin_10;
    valve[8].port = GPIOB;
    valve[8].rcc = RCC_AHB1Periph_GPIOB;
    ValveDriver_Init(&valve[8]);

    valve[9].pin = GPIO_Pin_11;
    valve[9].port = GPIOB;
    valve[9].rcc = RCC_AHB1Periph_GPIOB;
    ValveDriver_Init(&valve[9]);

    valve[10].pin = GPIO_Pin_12;
    valve[10].port = GPIOB;
    valve[10].rcc = RCC_AHB1Periph_GPIOB;
    ValveDriver_Init(&valve[10]);

    valve[11].pin = GPIO_Pin_13;
    valve[11].port = GPIOB;
    valve[11].rcc = RCC_AHB1Periph_GPIOB;
    ValveDriver_Init(&valve[11]);

    valve[12].pin = GPIO_Pin_14;
    valve[12].port= GPIOB;
    valve[12].rcc = RCC_AHB1Periph_GPIOB;
    ValveDriver_Init(&valve[12]);

    valve[13].pin = GPIO_Pin_15;
    valve[13].port = GPIOB;
    valve[13].rcc = RCC_AHB1Periph_GPIOB;
    ValveDriver_Init(&valve[13]);

    valve[14].pin = GPIO_Pin_8;
    valve[14].port= GPIOD;
    valve[14].rcc = RCC_AHB1Periph_GPIOD;
    ValveDriver_Init(&valve[14]);

    valve[15].pin = GPIO_Pin_9;
    valve[15].port = GPIOD;
    valve[15].rcc = RCC_AHB1Periph_GPIOD;
    ValveDriver_Init(&valve[15]);

    valve[16].pin = GPIO_Pin_10;
    valve[16].port= GPIOD;
    valve[16].rcc = RCC_AHB1Periph_GPIOD;
    ValveDriver_Init(&valve[16]);

    valve[17].pin = GPIO_Pin_11;
	valve[17].port= GPIOD;
	valve[17].rcc = RCC_AHB1Periph_GPIOD;
	ValveDriver_Init(&valve[17]);

	valve[18].pin = GPIO_Pin_12;
	valve[18].port= GPIOD;
	valve[18].rcc = RCC_AHB1Periph_GPIOD;
	ValveDriver_Init(&valve[18]);

	valve[19].pin = GPIO_Pin_13;
	valve[19].port= GPIOD;
	valve[19].rcc = RCC_AHB1Periph_GPIOD;
	ValveDriver_Init(&valve[19]);

	valve[20].pin = GPIO_Pin_14;
	valve[20].port= GPIOD;
	valve[20].rcc = RCC_AHB1Periph_GPIOD;
	ValveDriver_Init(&valve[20]);

	valve[21].pin = GPIO_Pin_15;
	valve[21].port= GPIOD;
	valve[21].rcc = RCC_AHB1Periph_GPIOD;
	ValveDriver_Init(&valve[21]);

	//球阀↑↑↑↑↑↑
	//夹管阀↓↓↓↓↓
	valve[22].pin = GPIO_Pin_3;
	valve[22].port = GPIOD;
	valve[22].rcc = RCC_AHB1Periph_GPIOD;
	ValveDriver_Init(&valve[22]);

	valve[23].pin = GPIO_Pin_4;
	valve[23].port = GPIOD;
	valve[23].rcc = RCC_AHB1Periph_GPIOD;
	ValveDriver_Init(&valve[23]);

	valve[24].pin = GPIO_Pin_5;
	valve[24].port = GPIOD;
	valve[24].rcc = RCC_AHB1Periph_GPIOD;
	ValveDriver_Init(&valve[24]);

	valve[25].pin = GPIO_Pin_6;
	valve[25].port= GPIOD;
	valve[25].rcc =  RCC_AHB1Periph_GPIOD;
	ValveDriver_Init(&valve[25]);

	valve[26].pin = GPIO_Pin_7;
	valve[26].port = GPIOD;
	valve[26].rcc = RCC_AHB1Periph_GPIOD;
	ValveDriver_Init(&valve[26]);

	valve[27].pin = GPIO_Pin_5;
	valve[27].port = GPIOB;
	valve[27].rcc = RCC_AHB1Periph_GPIOB;
	ValveDriver_Init(&valve[27]);

	valve[28].pin = GPIO_Pin_6;
	valve[28].port = GPIOB;
	valve[28].rcc = RCC_AHB1Periph_GPIOB;
	ValveDriver_Init(&valve[28]);

	valve[29].pin = GPIO_Pin_2;
	valve[29].port = GPIOE;
	valve[29].rcc = RCC_AHB1Periph_GPIOE;
	ValveDriver_Init(&valve[29]);

	valve[30].pin = GPIO_Pin_3;
	valve[30].port = GPIOE;
	valve[30].rcc = RCC_AHB1Periph_GPIOE;
	ValveDriver_Init(&valve[30]);

    for(i = 0; i < SOLENOIDVALVECONF_TOTALVAlVES; i++)
    {
        ValveDriver_Control(&valve[i], VAlVE_CLOSE);
    }

}


