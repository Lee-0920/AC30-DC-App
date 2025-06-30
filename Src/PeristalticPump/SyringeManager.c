/*
 * SyringeManager.c
 *
 *  Created on: 2021年1月14日
 *      Author: hyz
 */

#include "SyringeManager.h"
#include "SolenoidValve/ValveManager.h"
#include "SystemConfig.h"
#include "McuFlash.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "LuipApi/MotorControlInterface.h"
#include "DncpStack.h"
#include "Dncp/App/DscpSysDefine.h"

static xTaskHandle s_sy11Handle;
static void SyringeTask(void *argument);
static Bool SyringeManager_SetParam(float volume,float maxSpeed,float acceleration);
static SY11 s_sy11;
static Uint16 s_sy11Period = 200;

void SyringeManager_Init()
{
	memset(&s_sy11, 0, sizeof(s_sy11));
	s_sy11.curStep = 0;
	s_sy11.curVol = 0;
	s_sy11.targetStep = 0;
	s_sy11.status = SYRINGE_IDLE;
	s_sy11.isConnect = FALSE;
	s_sy11.totalStep = 0;
	s_sy11.isSendEvent = FALSE;
	s_sy11.targetValve = 1;
	s_sy11.timeout = (Uint16)60*(1000/s_sy11Period);
	s_sy11.timecnt = 0;
    xTaskCreate(SyringeTask, "SyringeTask", 128, NULL, 5, &s_sy11Handle);
}

void  SyringeManager_SendEvent(MoveResult moveResult)
{
    if(TRUE == s_sy11.isSendEvent)
    {
        Uint8 data[10] = {0};
        data[0] = SYRINGE_INDEX;
        memcpy(data + sizeof(Uint8), &moveResult, sizeof(moveResult));
        DncpStack_SendEvent(DSCP_EVENT_MCI_MOTOR_RESULT, data , sizeof(Uint8) + sizeof(moveResult));
        DncpStack_BufferEvent(DSCP_EVENT_MCI_MOTOR_RESULT, data , sizeof(Uint8) + sizeof(moveResult));
        Printf("#SY11 Event#");
    }
    s_sy11.isSendEvent = FALSE;
}

void SyringeManager_SendEventOpen(void)
{
	s_sy11.isSendEvent = TRUE;
	Printf("SY11 SendEventOpen\n");
}
void SyringeManager_Debug(Bool debug)
{
	if(debug)
	{
		vTaskResume(s_sy11Handle);
		Printf("SY11 Connect Start\n");
	}
	else
	{
		vTaskSuspend(s_sy11Handle);
		Printf("SY11 Connect Stop\n");
	}

}

float SyringeManager_GetFactor()
{
    return 0;
}

float SyringeManager_GetCurrentVol()
{
	float vol = (float)(s_sy11.curStep * SY11_FACTOR);
	Printf("SY11 Get Current Vol %f\n", vol);
    return vol;
}

Bool SyringeManager_SetFactor(float xfactor)
{
    return TRUE;
}

Uint32 SyringeManager_GetMaxAcceleration()
{
    return 0;
}

Bool SyringeManager_SetMaxAcceleration(Uint32 acc)
{
    return TRUE;
}

Bool SyringeManager_SetValveInput(Uint8 index)
{
	Bool ret = TRUE;
	Uint8 data[8] = {0};
	Uint16 reg = REG_VALVE_IO_INPUT;
	Uint16 code = index;
	if(!s_sy11.isConnect)
	{
		TRACE_INFO("SY11 Connect Failed\n");
		return FALSE;
	}
	s_sy11.reg = REG_VALVE_IO_INPUT;
	data[0] = DEVICE_ADDR;
	data[1] = FUNCTION_CODE_WRITE_16BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = code>>8;
	data[5] = code&0xFF;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[6] = crc16&0xFF;
	data[7] = crc16>>8&0xFF;
	Uint8 timeout = 10;
	s_sy11.timecnt = 0;
	while(timeout--)
	{
		if(!s_sy11.isSending)
		{
			Communication_Send(SYRINGE_INDEX, (char*)data, sizeof(data));
			s_sy11.totalStep = 0;
			TRACE_INFO("Syringe Set Valve Input Failed\n");
			ret = TRUE;
			s_sy11.status = SYRINGE_RESET;
			break;
		}
		System_Delay(1);
	}
	if(!ret)
	{
		TRACE_INFO("Syringe Valve Input Cmd Timeout\n");
	}
    return ret;
}


Bool SyringeManager_SetValveOutput(Uint8 index)
{
	Bool ret = TRUE;
	Uint8 data[8] = {0};
	Uint16 reg = REG_VALVE_IO_OUTPUT;
	Uint16 code = index;
	if(!s_sy11.isConnect)
	{
		TRACE_INFO("SY11 Connect Failed\n");
		return FALSE;
	}
	s_sy11.reg = REG_VALVE_IO_OUTPUT;
	data[0] = DEVICE_ADDR;
	data[1] = FUNCTION_CODE_WRITE_16BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = code>>8;
	data[5] = code&0xFF;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[6] = crc16&0xFF;
	data[7] = crc16>>8&0xFF;
	Uint8 timeout = 10;
	s_sy11.timecnt = 0;
	while(timeout--)
	{
		if(!s_sy11.isSending)
		{
			Communication_Send(SYRINGE_INDEX, (char*)data, sizeof(data));
			s_sy11.totalStep = 0;
			TRACE_INFO("Syringe Set Valve Output Failed\n");
			ret = TRUE;
			s_sy11.status = SYRINGE_RESET;
			break;
		}
		System_Delay(1);
	}
	if(!ret)
	{
		TRACE_INFO("Syringe Valve Output Cmd Timeout\n");
	}
    return ret;
}

Bool SyringeManager_Reset(Bool SendEvent)
{
	Bool ret = TRUE;
	Uint8 data[8] = {0};
	Uint16 reg = REG_RESET;
	Uint16 code = 1;
	if(!s_sy11.isConnect)
	{
		TRACE_INFO("SY11 Connect Failed\n");
		return FALSE;
	}
	s_sy11.reg = REG_RESET;
	data[0] = DEVICE_ADDR;
	data[1] = FUNCTION_CODE_WRITE_16BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = code>>8;
	data[5] = code&0xFF;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[6] = crc16&0xFF;
	data[7] = crc16>>8&0xFF;
	Uint8 timeout = 10;
	s_sy11.timecnt = 0;
	while(timeout--)
	{
		if(!s_sy11.isSending)
		{
			Communication_Send(SYRINGE_INDEX, (char*)data, sizeof(data));
			s_sy11.totalStep = 0;
			TRACE_INFO("Syringe Reset\n");
			ret = TRUE;
			s_sy11.status = SYRINGE_RESET;
			break;
		}
		System_Delay(1);
	}
	if(!ret)
	{
		TRACE_INFO("Syringe Reset Cmd Timeout\n");
	}
    return ret;
}

Bool SyringeManager_Start(Uint8 index, float volume, SY11Direction dir)
{
	Bool ret = TRUE;
//	Uint8 data[8] = {0};
	Uint16 reg;
	if(!s_sy11.isConnect)
	{
		TRACE_INFO("SY11 Connect Failed\n");
		return FALSE;
	}

	Uint16 code = (Uint32)(volume * 1.0 / (SY11_FACTOR));
	Printf("##%d, %f\n", code, volume);
	if(code>SY11_MAX_STEP)
	{
		TRACE_ERROR("SY11 Step Error %d\n", code);
		return FALSE;
	}
	s_sy11.targetValve = index;
	s_sy11.targetStep = code;
	s_sy11.curVol = volume;
	s_sy11.dir = dir;
	s_sy11.targetValve = index;
	if(dir == SY11_DRAIN)
	{
		reg = REG_MOVE_DRAIN;
		s_sy11.reg = REG_MOVE_DRAIN;
		if(s_sy11.totalStep < code)
		{
			TRACE_ERROR("SY11 Step Error %d, Curretn Step %d\n", code, s_sy11.totalStep);
			return FALSE;
		}
		else
		{
			s_sy11.totalStep -= code;
		}
	}
	else if(dir == SY11_ABS)
	{
		reg = REG_MOVE_ABS;
		s_sy11.reg = REG_MOVE_ABS;
		if(SY11_MAX_STEP < code)
		{
			TRACE_ERROR("SY11 Step Error %d, Max Step %d\n", code, SY11_MAX_STEP);
			return FALSE;
		}
		else
		{
			s_sy11.totalStep -= code;
		}
	}
	else
	{
		reg = REG_MOVE_SUCK;
		s_sy11.reg = REG_MOVE_SUCK;
		if(SY11_MAX_STEP < (s_sy11.totalStep + code))
		{
			TRACE_ERROR("SY11 Step Error %d, Max Step %d\n", code, SY11_MAX_STEP);
			return FALSE;
		}
		else
		{
			s_sy11.totalStep += code;
		}
	}
	s_sy11.status = SYRINGE_OPEN_VALVE_START;
	s_sy11.timecnt = 0;
	return ret;
}


Bool SyringeManager_Stop(void)
{
	Bool ret = TRUE;
	Uint8 data[8] = {0};
	Uint16 reg = REG_FORCED_STOP;
	Uint16 code = 1;
	if(!s_sy11.isConnect)
	{
		TRACE_INFO("SY11 Connect Failed\n");
		return FALSE;
	}
	s_sy11.reg = REG_FORCED_STOP;
	data[0] = DEVICE_ADDR;
	data[1] = FUNCTION_CODE_WRITE_16BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = code>>8;
	data[5] = code&0xFF;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[6] = crc16&0xFF;
	data[7] = crc16>>8&0xFF;
	Uint8 timeout = 10;
	while(timeout--)
	{
		if(!s_sy11.isSending)
		{
			Communication_Send(SYRINGE_INDEX, (char*)data, sizeof(data));
			TRACE_INFO("Syringe Stop\n");
			ret = TRUE;
			break;
		}
		System_Delay(1);
	}
	if(!ret)
	{
		TRACE_INFO("Syringe Stop Timeout\n");
	}
	return ret;
}

Bool SyringeManager_OpenValve(Uint8 num)
{
	Bool ret = FALSE;
	Uint8 data[8] = {0};
	Uint16 reg = REG_VALVE_CTL;
	Uint16 code = num;
	if(!s_sy11.isConnect)
	{
		TRACE_INFO("SY11 Connect Failed\n");
		return FALSE;
	}
	s_sy11.reg = REG_VALVE_CTL;
	data[0] = DEVICE_ADDR;
	data[1] = FUNCTION_CODE_WRITE_16BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = code>>8;
	data[5] = code&0xFF;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[6] = crc16&0xFF;
	data[7] = crc16>>8&0xFF;
	Uint8 timeout = 10;
	while(timeout--)
	{
		if(!s_sy11.isSending)
		{
			Communication_Send(SYRINGE_INDEX, (char*)data, sizeof(data));
			TRACE_INFO("Syringe Open Valve [%d]\n", num);
			s_sy11.status = SYRINGE_OPEN_VALVE_WAIT;
			ret = TRUE;
			break;
		}
		System_Delay(1);
	}
	if(!ret)
	{
		TRACE_INFO("Syringe Open Valve [%d] Timeout\n", num);
	}
	return ret;
}

Bool SyringeManager_SetParam(float volume,float maxSpeed,float acceleration)
{
	return TRUE ;
}

void SyringeManager_CheckConnect(void)
{
	Uint8 data[8] = {0};
	Uint16 reg = REG_STATUS_CODE;
	Uint16 code = 8;
	data[0] = DEVICE_ADDR;
	data[1] = FUNCTION_CODE_READ;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = code>>8;
	data[5] = code&0xFF;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[6] = crc16&0xFF;
	data[7] = crc16>>8&0xFF;
	s_sy11.isSending = TRUE; //发送标志
	Communication_Send(SYRINGE_INDEX, (char*)data, sizeof(data));
	System_Delay(10);
	s_sy11.isSending = FALSE;//发送标志
	Uint8 recdata[20] = {0};
	Communication_Read(SYRINGE_INDEX, (char*)recdata, sizeof(recdata));
	s_sy11.mscode = (recdata[3]<<8) | recdata[4];
	s_sy11.inputIO = (recdata[5]<<8) | recdata[6];
	s_sy11.mainVerion = (recdata[7]<<8) | recdata[8];
	s_sy11.subVersion = (recdata[9]<<8) | recdata[10];
	s_sy11.hisStep = (recdata[11]<<8) | recdata[12];
	s_sy11.hisValve = (recdata[13]<<8) | recdata[14];
	s_sy11.curStep = (recdata[15]<<8) | recdata[16];
	s_sy11.curValve = (recdata[17]<<8) | recdata[18];
	Communication_Clear(SYRINGE_INDEX);
	if(s_sy11.mscode & 0x60)
	{
		s_sy11.isConnect = TRUE;
	}
	else
	{
		TRACE_DEBUG("SY11 Connect Failed\n");
	}
	TRACE_DEBUG("#SY11# mscode %d, curStep %d, curValve %d\n",s_sy11.mscode, s_sy11.curStep, s_sy11.curValve);
}

void SY11_FlowManager(void)
{
	Uint8 data[8] = {0};
	Bool ret = FALSE;
	Uint16 crc16 = 0;
	Uint8 timeout = 10;
	switch(s_sy11.status)
	{
	case SYRINGE_OPEN_VALVE_START:
		data[0] = DEVICE_ADDR;
		data[1] = FUNCTION_CODE_WRITE_16BIT;
		data[2] = REG_VALVE_CTL>>8;
		data[3] = REG_VALVE_CTL&0xFF;
		data[4] = s_sy11.targetValve>>8;
		data[5] = s_sy11.targetValve&0xFF;
		crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
		data[6] = crc16&0xFF;
		data[7] = crc16>>8&0xFF;
		while(timeout--)
		{
			if(!s_sy11.isSending)
			{
				Communication_Send(SYRINGE_INDEX, (char*)data, sizeof(data));
				TRACE_INFO("\n ***********Open Valve %d, ", s_sy11.targetValve);
				s_sy11.status = SYRINGE_OPEN_VALVE_WAIT;
				ret = TRUE;
				break;
			}
			System_Delay(1);
		}
		if(!ret)
		{
			TRACE_INFO("Syringe Open Valve Timeout\n");
		}
		break;
	case SYRINGE_MOVE_START:
		Printf("#reg#%d\n", s_sy11.reg);
		data[0] = DEVICE_ADDR;
		data[1] = FUNCTION_CODE_WRITE_16BIT;
		data[2] = s_sy11.reg>>8;
		data[3] = s_sy11.reg&0xFF;
		data[4] = s_sy11.targetStep>>8;
		data[5] = s_sy11.targetStep&0xFF;
		crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
		data[6] = crc16&0xFF;
		data[7] = crc16>>8&0xFF;
		while(timeout--)
		{
			if(!s_sy11.isSending)
			{
				Communication_Send(SYRINGE_INDEX, (char*)data, sizeof(data));
				TRACE_INFO("\n ***********remainStep %d, dir %d,volume", s_sy11.targetStep, s_sy11.dir);
				System_PrintfFloat(TRACE_LEVEL_INFO,  s_sy11.curVol, 3);
				s_sy11.status = SYRINGE_MOVE_TO_TARGET;
				ret = TRUE;
				break;
			}
			System_Delay(1);
		}
		if(!ret)
		{
			TRACE_INFO("Syringe Strat Timeout\n");
		}
		break;
	default:
		break;
	}
}

static void SyringeTask(void *argument)
{
	static Bool isReset = TRUE;
	while(1)
	{
		SyringeManager_CheckConnect();
		System_Delay(100);
		SY11_FlowManager();
		switch(s_sy11.status)
		{
			case SYRINGE_IDLE:
				if(isReset && s_sy11.isConnect) //下位机重启复位
				{
					System_Delay(500);
					SyringeManager_SetValveInput(1); //输入阀1
					System_Delay(100);
					SyringeManager_SetValveOutput(2);//输出阀2
					System_Delay(100);
					SyringeManager_Reset(FALSE);
					isReset = FALSE;
				}
				break;
			case SYRINGE_RESET:
				if(s_sy11.isConnect && s_sy11.curStep==0)
				{
					TRACE_INFO("SY11 Reset Done\n");
					s_sy11.status = SYRINGE_IDLE;
					if(s_sy11.isSendEvent)
					{
						SyringeManager_SendEvent(RESULT_FINISHED);
					}
				}
				else if(s_sy11.timecnt > s_sy11.timeout)
				{
					if(s_sy11.isSendEvent)
					{
						SyringeManager_SendEvent(RESULT_FAILED);
					}
					TRACE_INFO("SY11 Reset Timeout\n");
					s_sy11.status = SYRINGE_IDLE;
				}
				s_sy11.timecnt++;
				break;
			case SYRINGE_STOP:
				break;
			case SYRINGE_MOVE_TO_TARGET:
				if(s_sy11.isConnect && s_sy11.curStep==s_sy11.totalStep)
				{
					if(s_sy11.isSendEvent)
					{
						SyringeManager_SendEvent(RESULT_FINISHED);
					}
					TRACE_INFO("SY11 Mov Done\n");
					s_sy11.status = SYRINGE_IDLE;
					s_sy11.targetStep = 0;
				}
				else if(s_sy11.timecnt > s_sy11.timeout)
				{
					if(s_sy11.isSendEvent)
					{
						SyringeManager_SendEvent(RESULT_FAILED);
					}
					TRACE_INFO("SY11 Mov To Target Timeout\n");
					s_sy11.status = SYRINGE_IDLE;
				}
				s_sy11.timecnt++;
				break;
			case SYRINGE_OPEN_VALVE_WAIT:
				if(s_sy11.isConnect && s_sy11.curValve==s_sy11.targetValve)
				{
					TRACE_INFO("SY11 Valve Open Done\n");
					if(s_sy11.targetStep>0)
					{
						s_sy11.status = SYRINGE_MOVE_START;
						TRACE_INFO("SY11 Switch To Start %d\n", s_sy11.targetStep);
					}
					else
					{
						s_sy11.status = SYRINGE_IDLE;
					}
				}
				else if(s_sy11.timecnt > s_sy11.timeout)
				{
					if(s_sy11.isSendEvent)
					{
						SyringeManager_SendEvent(RESULT_FAILED);
					}
					TRACE_INFO("SY11 Open Valve Timeout\n");
					s_sy11.status = SYRINGE_IDLE;
				}
				s_sy11.timecnt++;
				break;
			default:
				break;
		}
		vTaskDelay(s_sy11Period / portTICK_RATE_MS);
	}
}
