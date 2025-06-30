/*
 * InfrSensor.h
 *
 *  Created on: 2024年6月19日
 *      Author: admin
 */

#ifndef SRC_DRIVER_LIQUIDDRIVER_RELAY_H_
#define SRC_DRIVER_LIQUIDDRIVER_RELAY_H_
#include "stm32f4xx.h"
#include "Common/Types.h"
#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    GPIO_TypeDef *port;
    Uint16 pin;
    uint32_t rcc;
} RelayDriver;

typedef enum
{
    RELAY_IDLE,
    RELAY_BUSY,
	MAX_STATUS
} RelayStatus;

void Realy_Init(RelayDriver *relay);
void Relay_On(RelayDriver *relay);
void Relay_Off(RelayDriver *relay);
RelayStatus Realy_Read(RelayDriver *relay);

#ifdef __cplusplus
}
#endif
#endif /* SRC_DRIVER_LIQUIDDRIVER_RELAY_H_ */
