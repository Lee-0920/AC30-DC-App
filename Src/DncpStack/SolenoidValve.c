/**
 * @file SolenoidValveInterface.h
 * @brief 电磁阀控制执行文件。
 * @version 1.0.0
 * @author xingfan
 * @date 2016-05-27
 */
#include "Tracer/Trace.h"
#include "Dncp/App/DscpSysDefine.h"
#include <string.h>
#include "SolenoidValve/ValveManager.h"
#include "SolenoidValve.h"
#include "Peripheral/DetectSensorManager.h"
#include "Driver/TempDriver/TempADCollect.h"
#include "Driver/TempDriver/EnvTempCollect.h"
/**
 * @brief 查询系统支持的总电磁阀数目。
 * @param dscp
 * @param data
 * @param len
 */
void SolenoidVale_GetTotalValves(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 TotalValves = ValveManager_GetTotalValves();
    DscpDevice_SendResp(dscp, &TotalValves, sizeof(TotalValves));
}
/**
 * @brief 查询当前开启的阀门映射图。
 * @param dscp
 * @param data
 * @param len
 */
void SolenoidVale_GetValvesMap(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint32 map = ValveManager_GetValvesMap();
    TRACE_MARK("\nGetMap: 0x%x\n",map);
    DscpDevice_SendResp(dscp, &map, sizeof(Uint32));
}
/**
 * @brief 设置要开启的阀门映射图。
 * @param dscp
 * @param data
 * @param len
 */
void SolenoidVale_SetValvesMap(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint32 map =  0;
    Uint16 ret = DSCP_OK;

    memcpy(&map, data, sizeof(Uint32));

    if (FALSE == ValveManager_SetValvesMap(map))
    {
        ret = DSCP_ERROR;
    }
    DscpDevice_SendStatus(dscp, ret);
}
/**
 * @brief 查询当前传感器映射图。
 * @param dscp
 * @param data
 * @param len
 */
void SolenoidVale_GetLiquidSensorsMap(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint8 map = 0;
    //开门传感器
    if(DetectSensorManager_GetSensor(0))
    {
    	map |= 1<<0;
    }
    //原水传感器
    if(DetectSensorManager_GetSensor(1))
	{
    	map |= 1<<1;
	}
    //试剂传感器
    if(TempADCollect_GetAD(2) < 3800)
	{
    	map |= 1<<2;
	}
    //漏液传感器1
    if(TempADCollect_GetAD(0) < 1000)
	{
		map |= 1<<3;
	}
    //漏液传感器2
    if(TempADCollect_GetAD(1) < 1000)
	{
		map |= 1<<4;
	}

    TRACE_MARK("\nGetLiquidMap: 0x%x\n",map);
    DscpDevice_SendResp(dscp, &map, sizeof(Uint8));
}

/**
 * @brief 查询当前温度。
 * @return 当前温度，包括控制室温度和环境温度，格式如下：
 *     - thermostatTemp Float32，恒温室温度，单位为摄氏度。
 */
void SolenoidVale_GetTemperature(DscpDevice* dscp, Byte* data, Uint16 len)
{
    float temperature = EnvironmentTemp_Get();
    // 发送回应
    DscpDevice_SendResp(dscp, (void *) &temperature, sizeof(temperature));
}


