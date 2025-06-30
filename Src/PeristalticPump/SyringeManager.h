/*
 * SyringeManager.h
 *
 *  Created on: 2025年5月15日
 *      Author: Lee
 */

#ifndef SRC_DRIVER_SYRINGEMANAGER_H_
#define SRC_DRIVER_SYRINGEMANAGER_H_

#include "DNCP/App/DscpSysDefine.h"
#include "System.h"
#include "Common/Types.h"
#include "string.h"
#include "tracer/trace.h"
#include "Common/Types.h"
#include "Driver/UartDriver/Communication.h"

//设备地址
#define DEVICE_ADDR 	  						0x01 	    	//设备地址 默认值1(可通过旋钮修改地址)
#define SYRINGE_INDEX							3				//RS485串口索引值

//注射器参数
#define SY11_MAX_STEP							24000			//最大行程 单位：步
#define SY11_MAX_MM								60				//最大行程 单位：mm
#define SY11_MAX_VOL							1000			//最大行程 5ml 单位：uL
#define SY11_FACTOR								0.0417		//单步体积 单位：uL/步

/********保持寄存器(RW)**********/
#define REG_BROADCAST_ADDR 	  					0x00 	    	//组播 默认值0
#define REG_FORCED_STOP 	  					0x01 	    	//强停 默认值0 0-1
#define REG_RESET 	  							0x02 	    	//复位 默认值0 0-1
#define REG_VALVE_CTL	  						0x03 	    	//阀控制 默认值0 16bit数据：bit15:0-[无效],1-[最优路径]; bit14:0[正转],1[反转];bit0-13:1-最大孔位
																//例如0x800B,逆时针，预期最优路径转到11号孔
#define REG_MOVE_ABS							0x04 	    	//泵控制(绝对位置) 默认值0, 0-最大行程
#define REG_MOVE_SUCK							0x05 	    	//泵控制抽液 	  默认值0, 0-最大行程
#define REG_MOVE_DRAIN							0x06 	    	//泵控制排液 	  默认值0, 0-最大行程
#define REG_IO_CTL								0x07 	    	//IO输出 	 	  默认值0, 0-127
#define REG_SPEED_START							0x08 	    	//启动速度 	 	  默认值900, 单位：Hz
#define REG_SPEED_MAX							0x09 	    	//启动速度 	 	  默认值1400, 单位：Hz
#define REG_SPEED_END							0x0A 	    	//截至速度 	 	  默认值900, 单位：Hz
#define REG_ACCCELERATION						0x0B 	    	//加速度代号	 	  默认值14(35000KHz)
#define REG_ZERO_OFFSET							0x0C 	    	//零点偏移 	 	  默认值50, 单位：半步
#define REG_RETURN_GAP							0x0D 	    	//回程间隙 	 	  默认值12, 单位：半步
#define REG_RUN_CURRENT							0x0E 	    	//运行电流 	 	  默认值25, 1-31
#define REG_RESET_CURRENT						0x0F 	    	//复位电流 	 	  默认值25, 1-31
#define REG_VALVE_IO_INPUT						0x10 	    	//阀输入口 	 	  默认值4, 1-最大端口号
#define REG_VALVE_IO_OUTPUT						0x11 	    	//阀输出口 	 	  默认值5, 1-最大端口号
#define REG_BAUD_RATE							0x12 	    	//波特率		 	  默认值0(9600), 0-4
#define REG_MODBUS_SELECT						0x13 	    	//Modbus协议类型	  默认值3(RS485+RTU), 0-3

/********保持寄存器(RO)**********/
#define REG_STATUS_CODE		  					0x03E8 	    	//状态码
#define REG_INPUT_IO_LEVEL	  					0x03E9 	    	//输入IO水平
#define REG_VERSION_MAIN	  					0x03EA 	    	//主版本
#define REG_VERSION_SUB	  						0x03EB 	    	//次版本
#define REG_PUMP_HISTORY_MOVE  					0x03EC 	    	//注射器历史位移
#define REG_VALVE_HISTORY_MOVE 					0x03ED 	    	//阀历史位移
#define REG_PUMP_POSITION	  					0x03EE 	    	//泵位置				单位：半步
#define REG_VALVE_POSITION	  					0x03EF 	    	//阀位置				单位：端口号,[1,n]正常位置,-1 未知位置
#define REG_FIRMWARE_CRC			  			0x03F0 	    	//固件校验码
#define REG_DATA_CRC			  				0x03F1 	    	//数据校验码
#define REG_HARDWARE_SN		  					0x03F2 	    	//硬件序列号
#define REG_PUMP_0_ABS_LOCATION					0x03F3 	    	//电机0绝对位置		单位：编码器计数-注射泵
#define REG_PUMP_1_ABS_LOCATION					0x03F4 	    	//电机1绝对位置		单位：编码器计数-切换阀(切孔)
#define REG_PUMP_2_ABS_LOCATION					0x03F5 	    	//电机2绝对位置		单位：编码器计数-切换阀(伸缩)
#define REG_PUMP_0_CLOCK_LOCATION				0x03F6 	    	//电机0时钟位置		单位：编码器计数-注射泵
#define REG_PUMP_1_CLOCK_LOCATION				0x03F7 	    	//电机1时钟位置		单位：编码器计数-切换阀(切孔)
#define REG_PUMP_2_CLOCK_LOCATION				0x03F8 	    	//电机2时钟位置		单位：编码器计数-切换阀(伸缩)
#define REG_PUMP_0_CALIBRATION_STATUS			0x03F9 	    	//电机0校准状态
#define REG_PUMP_1_CALIBRATION_STATUS			0x03FA 	    	//电机1校准状态
#define REG_PUMP_2_CALIBRATION_STATUS			0x03FB 	    	//电机2校准状态
#define REG_PUMP_0_ENCODER_STATUS				0x03FC 	    	//电机0编码器状态
#define REG_PUMP_1_ENCODER_STATUS				0x03FD 	    	//电机1编码器状态
#define REG_PUMP_2_ENCODER_STATUS				0x03FE 	    	//电机2编码器状态

/********Modbus协议类型**********/
typedef enum
{
	RS232_ASCII,
	RS232_RTU,
	RS485_ASCII,
	RS485_RTU,
}ModbusType;

/********波特率**********/
typedef enum
{
	Baud_Rate_9600,
	Baud_Rate_19200,
	Baud_Rate_38400,
	Baud_Rate_57600,
	Baud_Rate_115200,
}BaudRate;

/********加速度代号(单位KHz)**********/
typedef enum
{
	Acc_2500_KHz = 1,
	Acc_5000_KHz,
	Acc_7500_KHz,
	Acc_10000_KHz,
	Acc_12500_KHz,
	Acc_15000_KHz,
	Acc_17500_KHz,
	Acc_20000_KHz,
	Acc_22500_KHz,
	Acc_25000_KHz,
	Acc_27500_KHz,
	Acc_30000_KHz,
	Acc_32500_KHz,
	Acc_35000_KHz,
	Acc_37500_KHz,
	Acc_40000_KHz,
	Acc_42500_KHz,
	Acc_45000_KHz,
	Acc_47500_KHz,
	Acc_50000_KHz,
}AccelerationSlope;

/********编码器状态**********/
typedef enum
{
    Normal,					//正常
	Magnetic_Low,			//磁性低
	Magnetic_High,			//磁性高
	SPI_Error,				//SPI错误
}EncoderStatus;

/********错误码**********/
typedef enum
{
	Illegal_Function_Code = 1,	//非法功能码
	Illegal_Data_Addr,			//非法数据地址
	Illegal_Data_Value,			//非法数据值
	Slaver_Fault,				//从设备故障
	Slaver_Reply,				//设备应答 可轮询该报文确定是否完成处理
	Slaver_Busy,				//从设备忙
	CRC_Error = 8,				//奇偶校验错误
	Gateway_Path_Error = 10,	//网关路径不可用
	Gateway_Path_Not_Reply = 11,//网关设备未响应
}ErrorCode;

typedef enum
{
    SYRINGE_IDLE,					//空闲状态确认通信连接情况
    SYRINGE_RESET,					//复位
	SYRINGE_STOP,					//停止
	SYRINGE_MOVE_START,				//发送移动命令
    SYRINGE_MOVE_TO_TARGET,			//移动到目标
	SYRINGE_OPEN_VALVE_START,    	//发送开阀命令
	SYRINGE_OPEN_VALVE_WAIT,    	//等待开阀完成
}SyringeStatus;

typedef enum
{
    SY11_PUMP,					//抽
	SY11_DRAIN,					//排
	SY11_ABS,					//绝对位置
}SY11Direction;

typedef struct
{
    float acceleration;        //单位为 步/平方秒 没有细分
    float maxSpeed;           //单位为步/秒 没有细分（注:上位机发送过来的ml/秒的最大速度会在PumpManager上转换）
} SyringeMotorParam;

typedef struct
{
    float curVol;        	   //单位为 uL
    SY11Direction dir;         //方向
    Uint16 reg;		   		   //控制寄存器
    Uint16 totalStep;     	   //单位为 步 总运动步数
    Uint8 targetValve;		   //目标阀号, 1-4
    Uint16 targetStep;     	   //单位为 步 目标运动步数
    SyringeStatus status;	   //注射器状态
    Uint16 mscode;			   //状态码
    Uint16 inputIO;			   //IO
    Uint16 mainVerion;		   //主版本
    Uint16 subVersion;		   //次版本
    Uint16 hisStep;			   //泵历史位移
    Uint16 hisValve;		   //阀历史
    Uint16 curStep;     	   //泵位置
    Uint16 curValve;     	   //阀位置
    Bool isConnect;			   //连接状态
    __IO Bool isSending;       //发送状态
    Bool isSendEvent;		   //发送事件
    Uint16 timeout;
	Uint16 timecnt;
}SY11;

void SyringeManager_Init();
Bool SyringeManager_Reset(Bool SendEvent);
float SyringeManager_GetFactor();
Bool SyringeManager_SetFactor(float xfactor);
Bool SyringeManager_Start(Uint8 index, float volume, SY11Direction dir);
Bool SyringeManager_Stop(void);
float SyringeManager_GetCurrentVol(void);
Uint32 SyringeManager_GetMaxAcceleration();
Bool SyringeManager_SetMaxAcceleration(Uint32 acc);
Bool SyringeManager_OpenValve(Uint8 num);
void SyringeManager_SendEventOpen(void);
void SyringeManager_Debug(Bool isDebug);
#endif /* SRC_DRIVER_SYRINGEMANAGER_H_ */
