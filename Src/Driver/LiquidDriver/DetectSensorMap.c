/*
 * ValveMap.c
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */

#include <LiquidDriver/DetectSensorMap.h>
#include "stm32f4xx.h"
#include "Peripheral/DetectSensorManager.h"

void DetectSensorMap_Init(Sensor *sensor)
{
    sensor[0].pin = GPIO_Pin_1;
    sensor[0].port = GPIOD;
    sensor[0].rcc = RCC_AHB1Periph_GPIOD;
    DetectSensorDriver_Init(&sensor[0]);

    sensor[1].pin = GPIO_Pin_0;
    sensor[1].port = GPIOD;
    sensor[1].rcc = RCC_AHB1Periph_GPIOD;
    DetectSensorDriver_Init(&sensor[1]);
}


