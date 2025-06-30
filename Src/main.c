/**
 * @file main.c
 * @brief 
 * @details
 *
 * @version 1.0.0
 * @author xingfan
 * @date 2016-4-28
 */

#include "FreeRTOS.h"
#include "task.h"
#include "Tracer/Trace.h"
#include "stm32f4xx.h"
#include "System.h"
#include "console/Console.h"
#include "CmdLine.h"
#include "DncpStack/DncpStack.h"
#include "SolenoidValve/ValveManager.h"
#include "DncpStack/DeviceInfo.h"
#include "DeviceIndicatorLED.h"
#include "Watchdog.h"
#include "HardwareType.h"
#include "DncpStack/DeviceStatus.h"
#include "DeviceUpdate/UpdateHandle.h"
#include "CheckLeaking/CheckLeakingControl.h"
#include "Driver/UartDriver/Communication.h"
#include "PeristalticPump/SyringeManager.h"
#include <TempDriver/EnvTempCollect.h>
#include "Peripheral/DetectSensorManager.h"
#include "Driver/LiquidDriver/AD5175Driver.h"
#include "Driver/LiquidDriver/RelayMap.h"

int main(void)
{
	for(int i=0;i<250000 ;i++);
    System_Init();
    DeviceIndicatorLED_Init();
    Watchdog_Init();
    // 功能模块初始化
    Console_Init();
    CmdLine_Init();

    DncpStack_Init();

    HardwareType_Init();
    ValveManager_Init();		//阀初始化
    EnvTempCollect_Init();		//温度采集初始化
    CheckLeakingControl_Init(); //漏液检测
    DetectSensorManager_Init(); //传感器初始化
    RelayMap_Init();			//继电器初始化
    AD5175_Init();				//定量点信号调节初始化
    Communication_Init();		//RS485初始化
    SyringeManager_Init();		//注射器SY11初始化
    DeviceInfo_Init();
 	UpdateHandle_Init();
    DeviceStatus_ReportResetEvent(DEVICE_RESET_POWER_ON); // 报告复位事件
//    PumpManager_Reset();

    vTaskStartScheduler();

    /* Infinite loop */
    while (1)
    {
    }
}
#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

/**
 * @}
 */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
