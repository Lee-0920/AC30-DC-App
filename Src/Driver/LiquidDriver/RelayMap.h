/*
 * RelayMap.h
 *
 *  Created on: 2024年6月19日
 *      Author: admin
 */

#ifndef SRC_DRIVER_LIQUIDDRIVER_RELAYMAP_H_
#define SRC_DRIVER_LIQUIDDRIVER_RELAYMAP_H_

#include <LiquidDriver/RelayDriver.h>

#define MAX_RELAY_NUM 3

void RelayMap_Init(void);
Bool RelayMap_On(Uint8 index);
Bool RelayMap_Off(Uint8 index);
RelayStatus RelayMap_Read(Uint8 index);

#endif /* SRC_DRIVER_LIQUIDDRIVER_RELAYMAP_H_ */
