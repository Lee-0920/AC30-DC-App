/*
 * MotorControl.c
 *
 *  Created on: 2018年3月8日
 *      Author: LIANG
 */

#include "Tracer/Trace.h"
#include "Dncp/App/DscpSysDefine.h"
#include <string.h>
#include "MotorControl.h"
#include "Driver/UartDriver/Communication.h"
#include "PeristalticPump/SyringeManager.h"

void MotorControl_GetMotionParam(DscpDevice* dscp, Byte* data, Uint16 len)
{
	//保留接口，功能后续有需求可直接添加内容
}

void MotorControl_SetMotionParam(DscpDevice* dscp, Byte* data, Uint16 len)
{
	//保留接口，功能后续有需求可直接添加内容
}

void MotorControl_GetStatus(DscpDevice* dscp, Byte* data, Uint16 len)
{
   Uint16 ret = DSCP_IDLE;
   //保留接口，功能后续有需求可直接添加内容
   DscpDevice_SendStatus(dscp, ret);
}

void MotorControl_GetMaxPos(DscpDevice* dscp, Byte* data, Uint16 len)
{
	Motor3DPos pos ;
	pos = Communication_MotorGetXYZLimit();
	DscpDevice_SendResp(dscp, &pos, sizeof(Motor3DPos));
}

void MotorControl_GetCurrentPos(DscpDevice* dscp, Byte* data, Uint16 len)
{
	Motor3DPos pos ;
	pos = Communication_MotorGetXYZ();
    DscpDevice_SendResp(dscp, &pos, sizeof(Motor3DPos));
}

void MotorControl_Start(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    Uint16 size = 0;
    Uint8 index;
    Int32 steps;
    MotorMode mode;

    //设置数据正确性判断
    size = sizeof(Uint8) + sizeof(Int32) + sizeof(Uint8);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
        memcpy(&index, data, sizeof(Uint8));
        size = sizeof(Uint8);
        memcpy(&steps, data + size, sizeof(Int32));
        size += sizeof(Int32);
        memcpy(&mode, data + size, sizeof(MotorMode));

        Communication_SendEventOpen(index);
        ret = Communication_MotorStart(index, mode, steps);
    }
    DscpDevice_SendStatus(dscp, ret);
}

void MotorControl_StartXYZ(DscpDevice* dscp, Byte* data, Uint16 len)
{
	Uint16 ret = DSCP_OK;
	Uint16 size = 0;
	MotorMode mode;
	Motor3DPos pospack;

	//设置数据正确性判断
	size = sizeof(MotorMode) + sizeof(Motor3DPos);
	if ((len > size))
	{
		ret = DSCP_ERROR;
		TRACE_ERROR("Parame Len Error\n");
		TRACE_ERROR("%d \n", size);
	}
	else
	{
		memcpy(&mode, data, sizeof(MotorMode));
		memcpy(&pospack, data+1, sizeof(Motor3DPos));
		Communication_SendEventOpen(2);
		ret = Communication_MotorStartXYZ(mode, pospack);
		Printf("mode[%d] xpos %d, ypos %d, zpos %d\n", mode, pospack.xpos, pospack.ypos, pospack.zpos);
	}
	DscpDevice_SendStatus(dscp, ret);
}

void MotorControl_Stop(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    Communication_SendEventOpen(0);
    Communication_RequestStop(0);
    Communication_RequestStop(1);
    ret = Communication_RequestStop(2);

    DscpDevice_SendStatus(dscp, ret);
}

void MotorControl_Reset(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    Communication_SendEventOpen(0);
    Communication_SendEventOpen(1);
    ret =Communication_MotorResetXYZ();

    DscpDevice_SendStatus(dscp, ret);
}

void MotorControl_GetSensorStatus(DscpDevice* dscp, Byte* data, Uint16 len)
{
   Uint16 ret = DSCP_IDLE;
   //保留接口，功能后续有需求可直接添加内容
   DscpDevice_SendStatus(dscp, ret);
}

void MotorControl_SY11Start(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    Uint16 size = 0;
    Uint8 index;
    SY11Direction dir;
    float vol;

    //设置数据正确性判断
    size = sizeof(Uint8) + sizeof(SY11Direction) + sizeof(float);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
    	memcpy(&index, data, sizeof(Uint8));
        memcpy(&dir, data + 1, sizeof(SY11Direction));
        memcpy(&vol, data + 2, sizeof(float));

        SyringeManager_SendEventOpen();
        TRACE_INFO("SY11 Dncp Start valve %d, dir %d, vol %f\n", index, dir, vol);
        if(!SyringeManager_Start(index, vol, dir))
        {
        	ret = DSCP_ERROR;
        }
    }
    DscpDevice_SendStatus(dscp, ret);
}

void MotorControl_SY11Reset(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;

    SyringeManager_SendEventOpen();
    if(!SyringeManager_Reset(TRUE))
	{
		ret = DSCP_ERROR;
	}

    DscpDevice_SendStatus(dscp, ret);
}

void MotorControl_SY11Stop(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;

    SyringeManager_SendEventOpen();
    if(!SyringeManager_Stop())
	{
		ret = DSCP_ERROR;
	}

    DscpDevice_SendStatus(dscp, ret);
}

void MotorControl_SY11GetCurrentVol(DscpDevice* dscp, Byte* data, Uint16 len)
{
    float vol = SyringeManager_GetCurrentVol();

    DscpDevice_SendResp(dscp, &vol, sizeof(float));
}

void MotorControl_SetRunSpeed(DscpDevice* dscp, Byte* data, Uint16 len)
{
	Uint16 ret = DSCP_OK;
	Uint16 size = 0;
	Uint8 speed;

	//设置数据正确性判断
	size = sizeof(Uint8);
	if ((len > size))
	{
		ret = DSCP_ERROR;
		TRACE_ERROR("Parame Len Error\n");
		TRACE_ERROR("%d \n", size);
	}
	else
	{
		memcpy(&speed, data, sizeof(Uint8));
		if(speed > 15)
		{
			ret = DSCP_ERROR;
			Printf("Set Run Speed Error [%d]\n", speed);
		}
		else
		{
			Communication_SetRunSpeed(speed);
			Printf("Set Run Speed [%d]\n", speed);
		}
	}
	DscpDevice_SendStatus(dscp, ret);
}

void MotorControl_GetRunSpeed(DscpDevice* dscp, Byte* data, Uint16 len)
{
	Uint8 speed = Communication_GetRunSpeed();
    DscpDevice_SendResp(dscp, &speed, sizeof(speed));
}
