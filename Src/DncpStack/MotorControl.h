/*
 * MotorControl.h
 *
 *  Created on: 2018年3月8日
 *      Author: LIANG
 */

#ifndef SRC_DNCPSTACK_MOTORCONTROL_H_
#define SRC_DNCPSTACK_MOTORCONTROL_H_

#include "Common/Types.h"
#include "DNCP/App/DscpDevice.h"
#include "LuipApi/MotorControlInterface.h"

void MotorControl_GetMotionParam(DscpDevice* dscp, Byte* data, Uint16 len);
void MotorControl_SetMotionParam(DscpDevice* dscp, Byte* data, Uint16 len);
void MotorControl_GetStatus(DscpDevice* dscp, Byte* data, Uint16 len);
void MotorControl_GetMaxPos(DscpDevice* dscp, Byte* data, Uint16 len);
void MotorControl_GetCurrentPos(DscpDevice* dscp, Byte* data, Uint16 len);
void MotorControl_Start(DscpDevice* dscp, Byte* data, Uint16 len);
void MotorControl_StartXYZ(DscpDevice* dscp, Byte* data, Uint16 len);
void MotorControl_Stop(DscpDevice* dscp, Byte* data, Uint16 len);
void MotorControl_Reset(DscpDevice* dscp, Byte* data, Uint16 len);
void MotorControl_SetRunSpeed(DscpDevice* dscp, Byte* data, Uint16 len);
void MotorControl_GetRunSpeed(DscpDevice* dscp, Byte* data, Uint16 len);
void MotorControl_GetSensorStatus(DscpDevice* dscp, Byte* data, Uint16 len);
void MotorControl_SY11Start(DscpDevice* dscp, Byte* data, Uint16 len);
void MotorControl_SY11GetCurrentVol(DscpDevice* dscp, Byte* data, Uint16 len);
void MotorControl_SY11Reset(DscpDevice* dscp, Byte* data, Uint16 len);
void MotorControl_SY11Stop(DscpDevice* dscp, Byte* data, Uint16 len);

//命令入口，全局宏定义：每一条命令对应一个命令码和处理函数
#define CMD_TABLE_MOTORCONTROL \
    DSCP_CMD_ENTRY(DSCP_CMD_MCI_GET_MOTION_PARAM, MotorControl_GetMotionParam), \
    DSCP_CMD_ENTRY(DSCP_CMD_MCI_SET_MOTION_PARAM, MotorControl_SetMotionParam),\
    DSCP_CMD_ENTRY(DSCP_CMD_MCI_GET_MOTOR_STATUS, MotorControl_GetStatus),\
    DSCP_CMD_ENTRY(DSCP_CMD_MCI_GET_MOTOR_MAX_POS, MotorControl_GetMaxPos), \
    DSCP_CMD_ENTRY(DSCP_CMD_MCI_GET_MOTOR_CURRENT_POS, MotorControl_GetCurrentPos),\
    DSCP_CMD_ENTRY(DSCP_CMD_MCI_MOTOR_MOVE, MotorControl_Start),\
	DSCP_CMD_ENTRY(DSCP_CMD_MCI_MOTOR_MOVE_XYZ, MotorControl_StartXYZ),\
    DSCP_CMD_ENTRY(DSCP_CMD_MCI_MOTOR_STOP , MotorControl_Stop),\
    DSCP_CMD_ENTRY(DSCP_CMD_MCI_MOTOR_RESET, MotorControl_Reset),\
	DSCP_CMD_ENTRY(DSCP_CMD_MCI_SET_MOTOR_RUN_SPEED, MotorControl_SetRunSpeed),\
	DSCP_CMD_ENTRY(DSCP_CMD_MCI_GET_MOTOR_RUN_SPEED, MotorControl_GetRunSpeed),\
    DSCP_CMD_ENTRY(DSCP_CMD_MCI_GET_SENSOR_STATUS , MotorControl_GetSensorStatus),\
	DSCP_CMD_ENTRY(DSCP_CMD_MCI_SY11_STRAT , MotorControl_SY11Start),\
	DSCP_CMD_ENTRY(DSCP_CMD_MCI_SY11_STOP , MotorControl_SY11Stop),\
	DSCP_CMD_ENTRY(DSCP_CMD_MCI_SY11_RESET , MotorControl_SY11Reset),\
	DSCP_CMD_ENTRY(DSCP_CMD_MCI_GET_SY11_CURRENT_VOL , MotorControl_SY11GetCurrentVol)

#endif /* SRC_DNCPSTACK_MOTORCONTROL_H_ */
