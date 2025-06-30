/**
 * @addtogroup module_Driver
 * @{
 */

/**
 * @file
 * @brief 命令行 Uart 驱动。
 * @details 仅供串口命令行程序使用，见 @ref Console.c
 *  主要的 RS232 通信实现移至 Console 模块。
 * @version 1.0.0
 * @author kim.xiejinqiang
 * @date 2012-10-31
 */

#include "Communication.h"
#include "string.h"
#include "Tracer/Trace.h"
#include "Common/Types.h"
#include "System.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "SystemConfig.h"
#include "LuipApi/MotorControlInterface.h"
#include "DncpStack.h"
#include "Dncp/App/DscpSysDefine.h"

#define MAX_DATA_LEN	24

static char s_sendResult[5][MAX_DATA_LEN] = {0};
static uint8_t s_sendPost[5] = {0};
static uint8_t s_dataLen[5] = {0};
static uint8_t s_recPost[5] = {0};
static uint8_t s_recResult[5][MAX_DATA_LEN] = {0};
static uint8_t s_recTemp[5][MAX_DATA_LEN] = {0};
static RxStatus s_rxStatus[5] = {Rx_Idle, Rx_Idle, Rx_Idle, Rx_Idle, Rx_Idle};
static Uint16 s_period = 100;
static uint8_t s_sy11Rec[MAX_DATA_LEN] = {0};
static uint8_t s_sy11Temp[MAX_DATA_LEN] = {0};
static uint8_t s_speed = 7;//电机移动速度 档位 0-15，对应P5.060-P5.075数值，调速方式：1.修改对应寄存器值默认值；2.修改档位值(最大档位对应3000rpm)

static xTaskHandle s_modbusHandle;
static void Communication_ModbusTaskHandle(void *pvParameters);

static xTaskHandle s_monitorHandle;
static void Communication_MonitorTaskHandle(void *pvParameters);

static Motor s_motor[3];

/**
 * @brief Comunication 串口驱动初始化。
 */
void Communication_RS485_1_USART_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//打开引脚时钟,串口时钟
	RS485_Tx1_RX_CONFIG;
	RS485_Tx1_TX_CONFIG;
	RS485_Tx1_CLK_CONFIG;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = RS485_Tx1_TX_PIN | RS485_Tx1_RX_PIN;
	GPIO_Init(RS485_Tx1_TX_GPIO_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(RS485_Tx1_TX_GPIO_PORT, RS485_Tx1_TX_PinSource, RS485_Tx1_GPIO_AF);
	GPIO_PinAFConfig(RS485_Tx1_RX_GPIO_PORT, RS485_Tx1_RX_PinSource, RS485_Tx1_GPIO_AF);

	//配置串口工作模式
	USART_InitStructure.USART_BaudRate = RS485_Tx1_UART_BAUD;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = RS485_StopBits;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(RS485_Tx1, &USART_InitStructure);
	{
		uint16_t mantissa;
		uint16_t fraction;

		float temp = (float) (RS485_Tx1_PERIPHERAL_SYSCLK * 1000000) / (RS485_Tx1_UART_BAUD * 16);
		mantissa = (uint16_t) temp;
		fraction = (uint16_t) ((temp - mantissa) * 16);
		mantissa <<= 4;
		mantissa += fraction;
		RS485_Tx1->BRR = mantissa;
	}
	USART_Cmd(RS485_Tx1, ENABLE);
	USART_ITConfig(RS485_Tx1, USART_IT_RXNE, ENABLE);

	// 配置中断向量管理器的RS485_Tx1_IRQn的中断
	NVIC_InitStructure.NVIC_IRQChannel = RS485_Tx1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x08; //抢占优先级 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //响应优先级 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief Comunication 串口驱动初始化。
 */
void Communication_RS485_2_USART_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//打开引脚时钟,串口时钟
	RS485_Tx2_RX_CONFIG;
	RS485_Tx2_TX_CONFIG;
	RS485_Tx2_CLK_CONFIG;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = RS485_Tx2_TX_PIN | RS485_Tx2_RX_PIN;
	GPIO_Init(RS485_Tx2_TX_GPIO_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(RS485_Tx2_TX_GPIO_PORT, RS485_Tx2_TX_PinSource, RS485_Tx2_GPIO_AF);
	GPIO_PinAFConfig(RS485_Tx2_RX_GPIO_PORT, RS485_Tx2_RX_PinSource, RS485_Tx2_GPIO_AF);

	//配置串口工作模式
	USART_InitStructure.USART_BaudRate = RS485_Tx2_UART_BAUD;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = RS485_StopBits;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(RS485_Tx2, &USART_InitStructure);
	{
		uint16_t mantissa;
		uint16_t fraction;

		float temp = (float) (RS485_Tx2_PERIPHERAL_SYSCLK * 1000000) / (RS485_Tx2_UART_BAUD * 16);
		mantissa = (uint16_t) temp;
		fraction = (uint16_t) ((temp - mantissa) * 16);
		mantissa <<= 4;
		mantissa += fraction;
		RS485_Tx2->BRR = mantissa;
	}
	USART_Cmd(RS485_Tx2, ENABLE);
	USART_ITConfig(RS485_Tx2, USART_IT_RXNE, ENABLE);

	// 配置中断向量管理器的RS485_Tx2_IRQn的中断
	NVIC_InitStructure.NVIC_IRQChannel = RS485_Tx2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x08; //抢占优先级 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //响应优先级 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief Comunication 串口驱动初始化。
 */
void Communication_RS485_3_USART_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//打开引脚时钟,串口时钟
	RS485_Tx3_RX_CONFIG;
	RS485_Tx3_TX_CONFIG;
	RS485_Tx3_CLK_CONFIG;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = RS485_Tx3_TX_PIN | RS485_Tx3_RX_PIN;
	GPIO_Init(RS485_Tx3_TX_GPIO_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(RS485_Tx3_TX_GPIO_PORT, RS485_Tx3_TX_PinSource, RS485_Tx3_GPIO_AF);
	GPIO_PinAFConfig(RS485_Tx3_RX_GPIO_PORT, RS485_Tx3_RX_PinSource, RS485_Tx3_GPIO_AF);

	//配置串口工作模式
	USART_InitStructure.USART_BaudRate = RS485_Tx3_UART_BAUD;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = RS485_StopBits;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(RS485_Tx3, &USART_InitStructure);
	{
		uint16_t mantissa;
		uint16_t fraction;

		float temp = (float) (RS485_Tx3_PERIPHERAL_SYSCLK * 1000000) / (RS485_Tx3_UART_BAUD * 16);
		mantissa = (uint16_t) temp;
		fraction = (uint16_t) ((temp - mantissa) * 16);
		mantissa <<= 4;
		mantissa += fraction;
		RS485_Tx3->BRR = mantissa;
	}
	USART_Cmd(RS485_Tx3, ENABLE);
	USART_ITConfig(RS485_Tx3, USART_IT_RXNE, ENABLE);

	// 配置中断向量管理器的RS485_Tx3_IRQn的中断
	NVIC_InitStructure.NVIC_IRQChannel = RS485_Tx3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x08; //抢占优先级 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //响应优先级 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief Comunication 串口驱动初始化。
 */
void Communication_RS485_4_USART_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//打开引脚时钟,串口时钟
	RS485_Tx4_RX_CONFIG;
	RS485_Tx4_TX_CONFIG;
	RS485_Tx4_CLK_CONFIG;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = RS485_Tx4_TX_PIN | RS485_Tx4_RX_PIN;
	GPIO_Init(RS485_Tx4_TX_GPIO_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(RS485_Tx4_TX_GPIO_PORT, RS485_Tx4_TX_PinSource, RS485_Tx4_GPIO_AF);
	GPIO_PinAFConfig(RS485_Tx4_RX_GPIO_PORT, RS485_Tx4_RX_PinSource, RS485_Tx4_GPIO_AF);

	//配置串口工作模式
	USART_InitStructure.USART_BaudRate = RS485_Tx4_UART_BAUD;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = SY11_StopBits;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(RS485_Tx4, &USART_InitStructure);
	{
		uint16_t mantissa;
		uint16_t fraction;

		float temp = (float) (RS485_Tx4_PERIPHERAL_SYSCLK * 1000000) / (RS485_Tx4_UART_BAUD * 16);
		mantissa = (uint16_t) temp;
		fraction = (uint16_t) ((temp - mantissa) * 16);
		mantissa <<= 4;
		mantissa += fraction;
		RS485_Tx4->BRR = mantissa;
	}
	USART_Cmd(RS485_Tx4, ENABLE);
	USART_ITConfig(RS485_Tx4, USART_IT_RXNE, ENABLE);

	// 配置中断向量管理器的RS485_Tx4_IRQn的中断
	NVIC_InitStructure.NVIC_IRQChannel = RS485_Tx4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x08; //抢占优先级 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //响应优先级 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief Comunication 串口驱动初始化。
 */
void Communication_RS485_5_USART_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//打开引脚时钟,串口时钟
	RS485_Tx5_RX_CONFIG;
	RS485_Tx5_TX_CONFIG;
	RS485_Tx5_CLK_CONFIG;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = RS485_Tx5_TX_PIN | RS485_Tx5_RX_PIN;
	GPIO_Init(RS485_Tx5_TX_GPIO_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(RS485_Tx5_TX_GPIO_PORT, RS485_Tx5_TX_PinSource, RS485_Tx5_GPIO_AF);
	GPIO_PinAFConfig(RS485_Tx5_RX_GPIO_PORT, RS485_Tx5_RX_PinSource, RS485_Tx5_GPIO_AF);

	//配置串口工作模式
	USART_InitStructure.USART_BaudRate = RS485_Tx5_UART_BAUD;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = RS485_StopBits;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(RS485_Tx5, &USART_InitStructure);
	{
		uint16_t mantissa;
		uint16_t fraction;

		float temp = (float) (RS485_Tx5_PERIPHERAL_SYSCLK * 1000000) / (RS485_Tx5_UART_BAUD * 16);
		mantissa = (uint16_t) temp;
		fraction = (uint16_t) ((temp - mantissa) * 16);
		mantissa <<= 4;
		mantissa += fraction;
		RS485_Tx5->BRR = mantissa;
	}
	USART_Cmd(RS485_Tx5, ENABLE);
	USART_ITConfig(RS485_Tx5, USART_IT_RXNE, ENABLE);

	// 配置中断向量管理器的RS485_Tx5_IRQn的中断
	NVIC_InitStructure.NVIC_IRQChannel = RS485_Tx5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x08; //抢占优先级 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //响应优先级 0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief Comunication 485切换引脚初始化。
 */
void Communication_Switch_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//配置RS485收发切换引脚
	RS485_Tx1_DE_RE_GPIO_CLK_CONFIG;
	RS485_Tx2_DE_RE_GPIO_CLK_CONFIG;
	RS485_Tx3_DE_RE_GPIO_CLK_CONFIG;
	RS485_Tx4_DE_RE_GPIO_CLK_CONFIG;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Pin = RS485_Tx1_DE_RE_SWITCH_PIN;
	GPIO_Init(RS485_Tx1_DE_RE_SWITCH_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = RS485_Tx2_DE_RE_SWITCH_PIN;
	GPIO_Init(RS485_Tx2_DE_RE_SWITCH_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = RS485_Tx3_DE_RE_SWITCH_PIN;
	GPIO_Init(RS485_Tx3_DE_RE_SWITCH_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = RS485_Tx4_DE_RE_SWITCH_PIN;
	GPIO_Init(RS485_Tx4_DE_RE_SWITCH_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = RS485_Tx5_DE_RE_SWITCH_PIN;
	GPIO_Init(RS485_Tx5_DE_RE_SWITCH_PORT, &GPIO_InitStructure);

	Communication_RS485_SwitchToRx(RS485_INDEX_1);
	Communication_RS485_SwitchToRx(RS485_INDEX_2);
	Communication_RS485_SwitchToRx(RS485_INDEX_3);
	Communication_RS485_SwitchToRx(RS485_INDEX_4);
	Communication_RS485_SwitchToRx(RS485_INDEX_5);
}


void Communication_StopAllMotor(void)
{
	Communication_EmgsStop(0);
	Communication_EmgsStop(1);
	Communication_EmgsStop(2);
	Printf("Stop All Motor\n");
}
/**
 * @brief Comunication 驱动初始化。
 */
void Communication_Init(void)
{
	Communication_RS485_1_USART_Init();
	Communication_RS485_2_USART_Init();
	Communication_RS485_3_USART_Init();
	Communication_RS485_4_USART_Init();
	Communication_RS485_5_USART_Init();
	Communication_Switch_Init();

	memset(s_recTemp, 0 , sizeof(s_recTemp));
	memset(s_recResult, 0 , sizeof(s_recResult));
	memset(s_recPost, 0 , sizeof(s_recPost));
	memset(s_motor, 0 , sizeof(s_motor));

	for(Uint8 i = 0;i<MOTOR_TOTAL;i++)
	{
		s_motor[i].number = i;
		s_motor[i].currentSteps = 0;
		if(i == MOTOR_Z_NUM)
		{
			s_motor[i].maxSteps = 45 * 100000; //38圈
		}
		else
		{
			s_motor[i].maxSteps = 38 * 100000; //38圈
		}
		s_motor[i].status = MOTOR_INIT;
		s_motor[i].mode = MOTOR_MOVE_ABSOLUTE_MODE;
		s_motor[i].isSendEvent = FALSE;
		s_motor[i].isRequestStop = FALSE;
		s_motor[i].isSwitchToStart = FALSE;
		s_motor[i].timeout = 60*(1000/s_period);
	}

	xTaskCreate(Communication_ModbusTaskHandle, "ModbusTaskHandle", 128, NULL,
	              		     5, &s_modbusHandle);
	xTaskCreate(Communication_MonitorTaskHandle, "MonitorTaskHandle", 256, NULL,
		              		     5, &s_monitorHandle);
}

/**
 * @brief RS485 1(USART2_IRQHandler)中断服务程序。
 */
void RS485_1_Hnadle(void)
{
    // 接收中断处理
    if (Communication_IsRxInterruptTriggered(RS485_INDEX_1))
    {
    	USART_ClearITPendingBit(RS485_Tx1, USART_IT_RXNE);
    	uint16_t data;
    	data = USART_ReceiveData(RS485_Tx1);
    	char cmd = 0;
    	memcpy(&cmd,&data,sizeof(char));
    	TRACE_MARK("\n RS481_1 R: %x %d", cmd, s_recPost[0]);
    	if(s_rxStatus[0] == Rx_Idle)
    	{
    		s_rxStatus[0] = Rx_Receiving;
    	}
    	s_recResult[0][s_recPost[0]++] = cmd;

    	USART_ITConfig(RS485_Tx1, USART_IT_RXNE, ENABLE); // 使能接收中断
    }

    // 发送中断处理
    if (Communication_IsTxInterruptTriggered(RS485_INDEX_1))
    {
    	if(s_sendPost[0] <= s_dataLen[0])
    	{
    		TRACE_MARK("\n RS481_1: %x", s_sendResult[0][s_sendPost[0]]);
    		Communication_Write(RS485_INDEX_1, s_sendResult[0][s_sendPost[0]++]);
    	}
    	else
    	{
    		Communication_DisableTxInterrupt(RS485_INDEX_1);
    		Communication_RS485_SwitchToRx(RS485_INDEX_1);
    		s_sendPost[0] = 0;
    		s_recPost[0] = 0;
    	}
    }
}

/**
 * @brief RS485 2(USART3_IRQHandler)中断服务程序。
 */
void RS485_2_Hnadle(void)
{
    // 接收中断处理
    if (Communication_IsRxInterruptTriggered(RS485_INDEX_2))
    {
    	USART_ClearITPendingBit(RS485_Tx2, USART_IT_RXNE);
    	uint16_t data;
    	data = USART_ReceiveData(RS485_Tx2);
    	char cmd = 0;
    	memcpy(&cmd,&data,sizeof(char));
    	TRACE_MARK("\n RS481_2 R: %x", cmd);
    	if(s_rxStatus[1] == Rx_Idle)
		{
			s_rxStatus[1] = Rx_Receiving;
		}
		s_recResult[1][s_recPost[1]++] = cmd;

    	USART_ITConfig(RS485_Tx2, USART_IT_RXNE, ENABLE); // 使能接收中断
    }

    // 发送中断处理
    if (Communication_IsTxInterruptTriggered(RS485_INDEX_2))
    {
    	if(s_sendPost[1] <= s_dataLen[1])
    	{
    		TRACE_MARK("\n RS481_2: %x", s_sendResult[1][s_sendPost[1]]);
    		Communication_Write(RS485_INDEX_2, s_sendResult[1][s_sendPost[1]++]);
    	}
    	else
    	{
    		Communication_DisableTxInterrupt(RS485_INDEX_2);
    		Communication_RS485_SwitchToRx(RS485_INDEX_2);
    		s_sendPost[1] = 0;
    		s_recPost[1] = 0;
    	}
    }
}

/**
 * @brief RS485 2(UART4_IRQHandler)中断服务程序。
 */
void RS485_3_Hnadle(void)
{
    // 接收中断处理
    if (Communication_IsRxInterruptTriggered(RS485_INDEX_3))
    {
    	USART_ClearITPendingBit(RS485_Tx3, USART_IT_RXNE);
    	uint16_t data;
    	data = USART_ReceiveData(RS485_Tx3);
    	char cmd = 0;
    	memcpy(&cmd,&data,sizeof(char));
    	TRACE_MARK("\n RS481_3 R: %x", cmd);

    	if(s_rxStatus[2] == Rx_Idle)
		{
			s_rxStatus[2] = Rx_Receiving;
		}
		s_recResult[2][s_recPost[2]++] = cmd;

    	USART_ITConfig(RS485_Tx3, USART_IT_RXNE, ENABLE); // 使能接收中断
    }

    // 发送中断处理
    if (Communication_IsTxInterruptTriggered(RS485_INDEX_3))
    {
    	if(s_sendPost[2] <= s_dataLen[2])
    	{
    		TRACE_MARK("\n RS481_3: %x", s_sendResult[2][s_sendPost[2]]);
    		Communication_Write(RS485_INDEX_3, s_sendResult[2][s_sendPost[2]++]);
    	}
    	else
    	{
    		Communication_DisableTxInterrupt(RS485_INDEX_3);
    		Communication_RS485_SwitchToRx(RS485_INDEX_3);
    		s_sendPost[2] = 0;
    		s_recPost[2] = 0;
    	}
    }
}

/**
 * @brief RS485 4(UART8_IRQHandler)中断服务程序。
 */
void RS485_4_Hnadle(void)
{
    // 接收中断处理
    if (Communication_IsRxInterruptTriggered(RS485_INDEX_4))
    {
    	USART_ClearITPendingBit(RS485_Tx4, USART_IT_RXNE);
    	uint16_t data;
    	data = USART_ReceiveData(RS485_Tx4);
    	char cmd = 0;
    	memcpy(&cmd,&data,sizeof(char));
    	TRACE_MARK("\n RS481_4 R: %x %d", cmd, s_recPost[3]);

    	if(s_rxStatus[3] == Rx_Idle)
		{
			s_rxStatus[3] = Rx_Receiving;
		}
//		s_recResult[3][s_recPost[3]++] = cmd;
    	s_sy11Rec[s_recPost[3]++] = cmd;

    	USART_ITConfig(RS485_Tx4, USART_IT_RXNE, ENABLE); // 使能接收中断
    }

    // 发送中断处理
    if (Communication_IsTxInterruptTriggered(RS485_INDEX_4))
    {

    	if(s_sendPost[3] <= s_dataLen[3])
    	{
    		TRACE_MARK("\n RS481_4: %x", s_sendResult[3][s_sendPost[3]]);
    		Communication_Write(RS485_INDEX_4, s_sendResult[3][s_sendPost[3]++]);
    	}
    	else
    	{
    		Communication_DisableTxInterrupt(RS485_INDEX_4);
    		Communication_RS485_SwitchToRx(RS485_INDEX_4);
    		s_sendPost[3] = 0;
    		s_recPost[3] = 0;
    	}
    }
}

/**
 * @brief RS485 4(UART8_IRQHandler)中断服务程序。
 */
void RS485_5_Hnadle(void)
{
	// 接收中断处理
	if (Communication_IsRxInterruptTriggered(RS485_INDEX_5))
	{
		USART_ClearITPendingBit(RS485_Tx5, USART_IT_RXNE);
		uint16_t data;
		data = USART_ReceiveData(RS485_Tx5);
		char cmd = 0;
		memcpy(&cmd,&data,sizeof(char));
		TRACE_MARK("\n RS485_5 R: %x", cmd);

		if(s_rxStatus[4] == Rx_Idle)
		{
			s_rxStatus[4] = Rx_Receiving;
		}
		s_recResult[4][s_recPost[4]++] = cmd;

		USART_ITConfig(RS485_Tx5, USART_IT_RXNE, ENABLE); // 使能接收中断
	}

	// 发送中断处理
	if (Communication_IsTxInterruptTriggered(RS485_INDEX_5))
	{

		if(s_sendPost[4] <= s_dataLen[4])
		{
			TRACE_MARK("\n RS485_5: %x", s_sendResult[4][s_sendPost[4]]);
			Communication_Write(RS485_INDEX_5, s_sendResult[4][s_sendPost[4]++]);
		}
		else
		{
			Communication_DisableTxInterrupt(RS485_INDEX_5);
			Communication_RS485_SwitchToRx(RS485_INDEX_5);
			s_sendPost[4] = 0;
    		s_recPost[4] = 0;
		}
	}
}

void Communication_Send(Uint8 index, char* Data, int len)
{
	memcpy(s_sendResult[index], Data, len);
	s_dataLen[index] = len;
	TRACE_MARK("MaxL [%d][%d]", index, len);
	Communication_RS485_SwitchToTx(index);
	System_Delay(1);
	Communication_EnableTxInterrupt(index);
}

void Communication_Read(Uint8 index, char* Data, int len)
{
	if(len>MAX_DATA_LEN)
	{
		len = MAX_DATA_LEN;
	}
//	memcpy(Data, s_recTemp[index]+1, len);
	memcpy(Data, s_sy11Temp+1, len);

}

void Communication_Clear(Uint8 index)
{
//	memset(&s_recTemp[index][0], 0, MAX_DATA_LEN);
	memset(s_sy11Temp, 0, MAX_DATA_LEN);
}

/**
- @brief 16进制Modbus CRC校验值计算
- @param buffer 需要数据
- @param len 数据长度
*/
Uint16 Communication_CRC(unsigned char *buffer,unsigned len)
{
    unsigned short wcrc = 0xFFFF;                   //16位CRC寄存器预置
    unsigned char temp;
    unsigned i = 0, j = 0;                      //计数
    for (i = 0; i < len; i++)                        //循环计算每个数据
    {
    temp = *buffer & 0x00FF;                    //将八位数据与crc寄存器亦或
    buffer++;                                   //指针地址增加，指向下个数据
    wcrc ^= temp;                               //将数据存入crc寄存器
    for (j = 0; j < 8; j++)                      //循环计算数据的
    {
    if (wcrc & 0x0001)                      //判断右移出的是不是1，如果是1则与多项式进行异或。
    {
    wcrc >>= 1;                           //先将数据右移一位
    wcrc ^= 0xA001;                     //与上面的多项式进行异或
    }
    else                                    //如果不是1，则直接移除
    {
    wcrc >>= 1;                           //直接移除
    }
    }
    }
    unsigned char CRC_L,CRC_H;                      //定义数据高低位
    CRC_L = wcrc & 0xFF;                            //CRC的低8位
    CRC_H = wcrc >> 8;                                //CRC的高8位
    return(( CRC_H << 8) | CRC_L);
}

/*
 * 软件强制Servo On
 *
 *启动伺服
 *Tx:036-7F 06 02 3C 00 01 83 A0
 *Rx:037-7F 06 02 3C 00 01 83 A0
 *关闭伺服
 *Tx:048-7F 06 02 3C 00 00 42 60
 *Rx:049-7F 06 02 3C 00 00 42 60
 * */
void Communication_ServoOn(Uint8 index)
{
	Uint8 data[8] = {0};
	Uint16 reg = REG_ASSISTED_FUNCTION;
	Uint16 code = ASSISTED_FORCED_SERVO_ON;
	data[0] = MOBUDS_ADDR;
	data[1] = FUNCTION_CODE_WRITE_16BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = code>>8;
	data[5] = code&0xFF;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[6] = crc16&0xFF;
	data[7] = crc16>>8&0xFF;
	Communication_Send(index, (char*)data, sizeof(data));
}

void Communication_ServoOff(Uint8 index)
{
	Uint8 data[8] = {0};
	Uint16 reg = REG_ASSISTED_FUNCTION;
	Uint16 code = ASSISTED_DEFAULT;
	data[0] = MOBUDS_ADDR;
	data[1] = FUNCTION_CODE_WRITE_16BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = code>>8;
	data[5] = code&0xFF;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[6] = crc16&0xFF;
	data[7] = crc16>>8&0xFF;
	Communication_Send(index, (char*)data, sizeof(data));
}

/*
 * 设置复位模式
 * 模式选择：0-A HOMING_RESET_MODE_0 ~ HOMING_RESET_MODE_A
 * Z信号选择:	HOMING_Z_REV_FIND 反找Z信号
 * 触碰极限机制：HOMING_LIMIT_ERROR 遇极限报错
 * */
void Communication_SetHomeMode(Uint8 index, Uint8 mode)
{
	Uint8 data[8] = {0};
	Uint16 reg = REG_HOMING_MODE;
	Uint16 code = HOMING_LIMIT_ERROR | HOMING_Z_REV_FIND | HOMING_RESET_MODE_0 | mode;
	data[0] = MOBUDS_ADDR;
	data[1] = FUNCTION_CODE_WRITE_16BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = code>>8;
	data[5] = code&0xFF;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[6] = crc16&0xFF;
	data[7] = crc16>>8&0xFF;
	Communication_Send(index, (char*)data, sizeof(data));
}

/*
 * 设置DI7为ORGP信号
 * */
void Communication_SetORGP(Uint8 index)
{
	Uint8 data[8] = {0};
	Uint16 reg = 0x0220;
	Uint16 code = 0x0124;
	data[0] = MOBUDS_ADDR;
	data[1] = FUNCTION_CODE_WRITE_16BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = code>>8;
	data[5] = code&0xFF;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[6] = crc16&0xFF;
	data[7] = crc16>>8&0xFF;
	Communication_Send(index, (char*)data, sizeof(data));
}


/*
 * 原点复位一阶段复位速度(32bit)
 * 默认1000 单位：0.1rpm, 输入限制：1-20000
 * */
void Communication_SetHomingFirstSpeed(Uint8 index, Uint32 speed)
{
	Uint8 data[13] = {0};
	Uint16 reg = REG_HOMING_FIRST_SPEED;
	Uint32 code = speed;
	if(code > 5000)
	{
		code = 5000;
		Printf("First Speed Over Limited\n");
	}
	data[0] = MOBUDS_ADDR;
	data[1] = FUNCTION_CODE_WRITE_32BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = 0x00;
	data[5] = 0x02;
	data[6] = 0x04;
	data[7] = code>>8;
	data[8] = code&0xFF;
	data[9] = code>>24;
	data[10] = code>>16;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[11] = crc16&0xFF;
	data[12] = crc16>>8&0xFF;
	Communication_Send(index, (char*)data, sizeof(data));
}

/*
 * 原点复位二阶段复位速度(32bit)
 * 默认200 单位：0.1rpm, 输入限制：1-5000
 * */
void Communication_SetHomingSecondSpeed(Uint8 index, Uint32 speed)
{
	Uint8 data[13] = {0};
	Uint16 reg = REG_HOMING_SECOND_SPEED;
	Uint32 code = speed;
	if(code > 5000)
	{
		code = 5000;
		Printf("Second Speed Over Limited\n");
	}
	data[0] = MOBUDS_ADDR;
	data[1] = FUNCTION_CODE_WRITE_32BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = 0x00;
	data[5] = 0x02;
	data[6] = 0x04;
	data[7] = code>>8;
	data[8] = code&0xFF;
	data[9] = code>>24;
	data[10] = code>>16;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[11] = crc16&0xFF;
	data[12] = crc16>>8&0xFF;
	Communication_Send(index, (char*)data, sizeof(data));
}

/*
 * 保存参数到EPPROM
 *
 * */
void Communication_SaveToEPPROM(Uint8 index)
{
	Uint8 data[8] = {0};
	Uint16 reg = REG_ASSISTED_FUNCTION;
	Uint16 code = ASSISTED_BACKUP_EEPROM;
	data[0] = MOBUDS_ADDR;
	data[1] = FUNCTION_CODE_WRITE_16BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = code>>8;
	data[5] = code&0xFF;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[6] = crc16&0xFF;
	data[7] = crc16>>8&0xFF;
	Communication_Send(index, (char*)data, sizeof(data));
}

void Communication_SetRunSpeed(Uint8 speed)
{
	if(speed > 15)
	{
		s_speed = 7;
		Printf("Run Speed Out Of Limited\n");
	}
	else
	{
		s_speed = speed;
		Printf("Set Run Speed %d \n", speed);
	}
}

Uint8 Communication_GetRunSpeed(void)
{
	return s_speed;
}

/*
 * 原点复位定义(32bit)
 * 复位完成停止，默认速度，默认延迟时间，默认上电不主动复位
 * */
void Communication_HomingDefine(Uint8 index)
{
	Uint8 data[13] = {0};
	Uint16 reg = REG_HOMING_DEFINE;
	Uint32 code = HOMING_DEFINE_FINISH_STOP | HOMING_DEFINE_ACC_0 | HOMING_DEFINE_DEC1_0
			| HOMING_DEFINE_DEC2_0 | HOMING_DEFINE_DLY_0 | HOMING_DEFINE_BOOT_OFF;
	data[0] = MOBUDS_ADDR;
	data[1] = FUNCTION_CODE_WRITE_32BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = 0x00;
	data[5] = 0x02;
	data[6] = 0x04;
	data[7] = code>>8;
	data[8] = code&0xFF;
	data[9] = code>>24;
	data[10] = code>>16;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[11] = crc16&0xFF;
	data[12] = crc16>>8&0xFF;
	Communication_Send(index, (char*)data, sizeof(data));
}

/*
 * 原点复位(16bit)
 * 触发命令：PR#0
 * */
Bool Communication_ResetAxis(Uint8 index)
{
	Uint8 data[8] = {0};
	Uint16 reg = REG_PR_CMD_TRIGGER;
	Uint16 code = PR_CMD_TRIGGER_RESET;
	data[0] = MOBUDS_ADDR;
	data[1] = FUNCTION_CODE_WRITE_16BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = code>>8;
	data[5] = code&0xFF;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[6] = crc16&0xFF;
	data[7] = crc16>>8&0xFF;
	Communication_Send(index, (char*)data, sizeof(data));
	Uint16 timeout = 100;
	Bool isTimeout = TRUE;
	while(timeout--)
	{
		if(s_rxStatus[index] == Rx_End)
		{
			isTimeout = FALSE;
			break;
		}
		System_Delay(1);
	}
	if(!isTimeout)
	{
		return TRUE;
	}
	return FALSE;
}

/*
 * 紧急停止(16bit)
 * 触发命令：PR#1000
 * */
void Communication_EmgsStop(Uint8 index)
{
	Uint8 data[8] = {0};
	Uint16 reg = REG_PR_CMD_TRIGGER;
	Uint16 code = PR_CMD_TRIGGER_STOP;
	data[0] = MOBUDS_ADDR;
	data[1] = FUNCTION_CODE_WRITE_16BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = code>>8;
	data[5] = code&0xFF;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[6] = crc16&0xFF;
	data[7] = crc16>>8&0xFF;
	Communication_Send(index, (char*)data, sizeof(data));
}

void Communication_TargetDefine(Uint8 index, Uint8 mode, Uint8 speed)
{
	Uint8 data[13] = {0};
	Uint16 reg = REG_PATH_1_DEFINE;
	Uint32 code = PATH_SINGLE | PATH_ACC_0 | PATH_DEC_0| PATH_DLY_0 |(mode<<6) |(speed<<16);
	data[0] = MOBUDS_ADDR;
	data[1] = FUNCTION_CODE_WRITE_32BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = 0x00;
	data[5] = 0x02;
	data[6] = 0x04;
	data[7] = code>>8;
	data[8] = code&0xFF;
	data[9] = code>>24;
	data[10] = code>>16;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[11] = crc16&0xFF;
	data[12] = crc16>>8&0xFF;
	Communication_Send(index, (char*)data, sizeof(data));
	Printf("Set [%d] TargetDefine %d\n", index, code);
}

void Communication_SetTargetParam(Uint8 index, Int32 param)
{
	Uint8 data[13] = {0};
	Uint16 reg = REG_PATH_1_DATA;
	Int32 code = param;
	data[0] = MOBUDS_ADDR;
	data[1] = FUNCTION_CODE_WRITE_32BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = 0x00;
	data[5] = 0x02;
	data[6] = 0x04;
	data[7] = code>>8;
	data[8] = code&0xFF;
	data[9] = code>>24;
	data[10] = code>>16;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[11] = crc16&0xFF;
	data[12] = crc16>>8&0xFF;
	Communication_Send(index, (char*)data, sizeof(data));
	Printf("Set [%d] TargetPram %d\n", index, code);
}

/*
 * 启动路径1(16bit)
 * 触发命令：PR#01
 * */
void Communication_MoveTarget(Uint8 index)
{
	Uint8 data[8] = {0};
	Uint16 reg = REG_PR_CMD_TRIGGER;
	Uint16 code = PR_CMD_TRIGGER_PATH_1;
	data[0] = MOBUDS_ADDR;
	data[1] = FUNCTION_CODE_WRITE_16BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = code>>8;
	data[5] = code&0xFF;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[6] = crc16&0xFF;
	data[7] = crc16>>8&0xFF;
	Communication_Send(index, (char*)data, sizeof(data));
}

/*
 * 设置PR模式
 * 模式选择：0-A REG_MODE_OPERATION
 * */
void Communication_SetPRMode(Uint8 index)
{
	Uint8 data[8] = {0};
	Uint16 reg = REG_MODE_OPERATION;
	Uint16 code = MODE_PR_CMD;
	data[0] = MOBUDS_ADDR;
	data[1] = FUNCTION_CODE_WRITE_16BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = code>>8;
	data[5] = code&0xFF;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[6] = crc16&0xFF;
	data[7] = crc16>>8&0xFF;
	Communication_Send(index, (char*)data, sizeof(data));
}

Int32 Communication_GetAxisPosition(Uint8 index)
{
	Int32 pos = 0;
	Uint8 data[8] = {0};
	Uint16 reg = REG_AXIS_POSITION_PUU;
	data[0] = MOBUDS_ADDR;
	data[1] = FUNCTION_CODE_READ;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = 0x00;
	data[5] = 0x02; //读两个寄存器
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[6] = crc16&0xFF;
	data[7] = crc16>>8&0xFF;
	Communication_Send(index, (char*)data, sizeof(data));
	Uint16 timeout = 100;
	Bool isTimeout = TRUE;
	while(timeout--)
	{
		if(s_rxStatus[index] == Rx_End)
		{
			isTimeout = FALSE;
			break;
		}
		System_Delay(1);
	}
	if(!isTimeout)
	{
		Uint16 LB = s_recTemp[index][4] | (s_recTemp[index][3]<<8);
		Uint16 HB = s_recTemp[index][6] | (s_recTemp[index][5]<<8);
		pos = (HB<<16) | LB;
//		Uint8 tmp[4] = {s_recTemp[index][6], s_recTemp[index][5], s_recTemp[index][4], s_recTemp[index][3]};
//		Printf(" %x %x %x %x\n",tmp[0],tmp[1],tmp[2],tmp[3]);
//		Printf("%x %x \n", HB, LB);
	}
//	Printf("Pos %d\n", pos);
	return pos;
}

void Communication_ClearAxisPosition(Uint8 index)
{
	Uint8 data[13] = {0};
	Uint16 reg = REG_AXIS_POSITION_PUU;
	Int32 code = 0;
	data[0] = MOBUDS_ADDR;
	data[1] = FUNCTION_CODE_WRITE_32BIT;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = 0x00;
	data[5] = 0x02;
	data[6] = 0x04;
	data[7] = code>>8;
	data[8] = code&0xFF;
	data[9] = code>>24;
	data[10] = code>>16;
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[11] = crc16&0xFF;
	data[12] = crc16>>8&0xFF;
	Communication_Send(index, (char*)data, sizeof(data));
	Printf("Clear Axis %d\n", code);
}
//void Communication_SetDIFunction(Uint8 index, Uint16 pram);
//Uint16 Communication_GetDIFunction(Uint8 index);

Uint16 Communication_GetDIStatus(Uint8 index)
{
	Uint16 ret = 0;
	Uint8 data[8] = {0};
	Uint16 reg = REG_DI_STATUS;
	data[0] = MOBUDS_ADDR;
	data[1] = FUNCTION_CODE_READ;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = 0x00;
	data[5] = 0x01; //读一个寄存器
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[6] = crc16&0xFF;
	data[7] = crc16>>8&0xFF;
	Communication_Send(index, (char*)data, sizeof(data));
	Uint16 timeout = 100;
	Bool isTimeout = TRUE;
	while(timeout--)
	{
		if(s_rxStatus[index] == Rx_End)
		{
			isTimeout = FALSE;
			break;
		}
		System_Delay(1);
	}
	if(!isTimeout)
	{
		ret = s_recTemp[index][4] | (s_recTemp[index][3]<<8);
	}
	Printf("DI Status [0x%x]\n", ret);
	return ret;
}

void Communication_DOReport(Uint16 arg)
{
	if(arg&DO_STATUS_SRDY)
	{
		Printf("DO_STATUS_SRDY\n");
	}
	if(arg&DO_STATUS_SON)
	{
		Printf("DO_STATUS_SON\n");
	}
	if(arg&DO_STATUS_ZSPD)
	{
		Printf("DO_STATUS_ZSPD\n");
	}
	if(arg&DO_STATUS_TSPD)
	{
		Printf("DO_STATUS_TSPD\n");
	}
	if(arg&DO_STATUS_TPOS)
	{
		Printf("DO_STATUS_TPOS\n");
	}
	if(arg&DO_STATUS_TQL)
	{
		Printf("DO_STATUS_TQL\n");
	}
	if(arg&DO_STATUS_ALRM)
	{
		Printf("DO_STATUS_ALRM\n");
	}
	if(arg&DO_STATUS_BRKR)
	{
		Printf("DO_STATUS_BRKR\n");
	}
	if(arg&DO_STATUS_HOME)
	{
		Printf("DO_STATUS_HOME\n");
	}
	if(arg&DO_STATUS_OLW)
	{
		Printf("DO_STATUS_OLW\n");
	}
	if(arg&DO_STATUS_WARNY)
	{
		Printf("DO_STATUS_WARNY\n");
	}
}

Uint16 Communication_GetDOStatus(Uint8 index)
{
	Uint16 ret = 0;
	Uint8 data[8] = {0};
	Uint16 reg = REG_DO_STATUS;
	data[0] = MOBUDS_ADDR;
	data[1] = FUNCTION_CODE_READ;
	data[2] = reg>>8;
	data[3] = reg&0xFF;
	data[4] = 0x00;
	data[5] = 0x01; //读一个寄存器
	Uint16 crc16 = Communication_CRC((unsigned char*)data, sizeof(data)-2);
	data[6] = crc16&0xFF;
	data[7] = crc16>>8&0xFF;
	Communication_Send(index, (char*)data, sizeof(data));
	Uint16 timeout = 100;
	Bool isTimeout = TRUE;
	while(timeout--)
	{
		if(s_rxStatus[index] == Rx_End)
		{
			isTimeout = FALSE;
			break;
		}
		System_Delay(1);
	}
	if(!isTimeout)
	{
		ret = s_recTemp[index][4] | (s_recTemp[index][3]<<8);
	}
	TRACE_MARK("DO Status [0x%x]\n", ret);
	return ret;
}

void Communication_SendEvent(Motor *motor, MoveResult moveResult)
{
    if(TRUE == motor->isSendEvent)
    {
        Uint8 data[10] = {0};
        data[0] = motor->number;
        memcpy(data + sizeof(Uint8), &moveResult, sizeof(moveResult));
        DncpStack_SendEvent(DSCP_EVENT_MCI_MOTOR_RESULT, data , sizeof(Uint8) + sizeof(moveResult));
        DncpStack_BufferEvent(DSCP_EVENT_MCI_MOTOR_RESULT, data , sizeof(Uint8) + sizeof(moveResult));
        Printf("#Motor %d Event#", motor->number);
    }
    motor->isSendEvent = FALSE;
}

void Communication_SendEventOpen(Uint8 index)
{
	s_motor[index].isSendEvent = TRUE;
	Printf("SendEventOpen\n");
}

Bool Communication_MotorStart(Uint8 index, Uint8 mode, Int32 steps)
{
	if(((MotorMode)mode == MOTOR_MOVE_ABSOLUTE_MODE) && (steps < 0))
	{
		TRACE_ERROR("Abs Mode Start Error [%d]->[%d]\n", s_motor[index].currentSteps, steps);
		return DSCP_BUSY;
	}
	else if(((MotorMode)mode == MOTOR_MOVE_RELATIVE_MODE) && (s_motor[index].currentSteps + steps < 0))
	{
		TRACE_ERROR("Relative Mode Start Error [%d]->[%d]\n", s_motor[index].currentSteps, steps);
		return DSCP_BUSY;
	}
	else if(((MotorMode)mode == MOTOR_MOVE_RELATIVE_MODE) && (s_motor[index].currentSteps + steps > s_motor[index].maxSteps))
	{
		TRACE_ERROR("Relative Mode Start Out Of Range [%d]->[%d]\n", s_motor[index].currentSteps, steps);
		return DSCP_BUSY;
	}
	else if(((MotorMode)mode == MOTOR_MOVE_ABSOLUTE_MODE) && (steps > s_motor[index].maxSteps))
	{
		TRACE_ERROR("Abs Mode Start Out Of Range [%d]->[%d]\n", s_motor[index].currentSteps, steps);
		return DSCP_BUSY;
	}
	s_motor[index].targetStep = steps;
	s_motor[index].mode = (MotorMode)mode;
	s_motor[index].isSwitchToStart = TRUE;
	s_motor[index].isXYZMode = FALSE;
	TRACE_INFO("Motor[%d] mode %d, target %d\n", index, s_motor[index].mode, s_motor[index].targetStep);
	return DSCP_OK;
}

Bool Communication_MotorStartXYZ(Uint8 mode, Motor3DPos mulpos)
{
	if(((MotorMode)mode == MOTOR_MOVE_ABSOLUTE_MODE) && (mulpos.xpos < 0 || mulpos.ypos < 0 || mulpos.zpos < 0))
	{
		TRACE_ERROR("Error Abs MotorXYZ , [x]%d-[y]%d-[z]%d\n", mulpos.xpos, mode, mulpos.ypos, mode, mulpos.zpos);
		return DSCP_BUSY;
	}
	else if(((MotorMode)mode == MOTOR_MOVE_RELATIVE_MODE) && (
			(s_motor[MOTOR_X_NUM].currentSteps + mulpos.xpos < 0) ||
			(s_motor[MOTOR_Y_NUM].currentSteps + mulpos.ypos < 0) ||
			(s_motor[MOTOR_Z_NUM].currentSteps + mulpos.zpos < 0)))
	{
		TRACE_ERROR("Error Rel MotorXYZ mode %d, [x]%d-[y]%d-[z]%d\n", mode, mulpos.xpos, mode, mulpos.ypos, mode, mulpos.zpos);
		return DSCP_BUSY;
	}
	else if(((MotorMode)mode == MOTOR_MOVE_RELATIVE_MODE) && (
			(s_motor[MOTOR_X_NUM].currentSteps + mulpos.xpos > s_motor[MOTOR_X_NUM].maxSteps) ||
			(s_motor[MOTOR_Y_NUM].currentSteps + mulpos.ypos > s_motor[MOTOR_Y_NUM].maxSteps) ||
			(s_motor[MOTOR_Z_NUM].currentSteps + mulpos.zpos > s_motor[MOTOR_Z_NUM].maxSteps)))
	{
		TRACE_ERROR("Relative Mode MotorXYZ Out Of Range [%d][%d][%d] + [%d][%d][%d]\n", s_motor[MOTOR_X_NUM].currentSteps,
														s_motor[MOTOR_Y_NUM].currentSteps,
														s_motor[MOTOR_Z_NUM].currentSteps,
														mulpos.xpos,mulpos.ypos,mulpos.zpos);
		return DSCP_BUSY;
	}
	else if(((MotorMode)mode == MOTOR_MOVE_ABSOLUTE_MODE) && (
			(mulpos.xpos > s_motor[MOTOR_X_NUM].maxSteps) ||
			(mulpos.ypos > s_motor[MOTOR_Y_NUM].maxSteps) ||
			(mulpos.zpos > s_motor[MOTOR_Z_NUM].maxSteps)))
	{
		TRACE_ERROR("Abs Mode MotorXYZ Out Of Range [%d][%d][%d]\n", mulpos.xpos, mulpos.ypos, mulpos.zpos);
		return DSCP_BUSY;
	}
	s_motor[MOTOR_X_NUM].targetStep = mulpos.xpos;
	s_motor[MOTOR_X_NUM].mode = (MotorMode)mode;
	s_motor[MOTOR_X_NUM].isSwitchToStart = FALSE;
	s_motor[MOTOR_X_NUM].status = MOTOR_TO_WAIT_Z_RESET;
	s_motor[MOTOR_X_NUM].isXYZMode = TRUE;
	s_motor[MOTOR_X_NUM].timecnt = 0;

	s_motor[MOTOR_Y_NUM].targetStep = mulpos.ypos;
	s_motor[MOTOR_Y_NUM].mode = (MotorMode)mode;
	s_motor[MOTOR_Y_NUM].isSwitchToStart = FALSE;
	s_motor[MOTOR_Y_NUM].status = MOTOR_TO_WAIT_Z_RESET;
	s_motor[MOTOR_Y_NUM].isXYZMode = TRUE;
	s_motor[MOTOR_Y_NUM].timecnt = 0;

	s_motor[MOTOR_Z_NUM].targetStep = mulpos.zpos;
	s_motor[MOTOR_Z_NUM].mode = (MotorMode)mode;
	s_motor[MOTOR_Z_NUM].isSwitchToStart = FALSE;
	s_motor[MOTOR_Z_NUM].status = MOTOR_TO_ZERO;
	s_motor[MOTOR_Z_NUM].isXYZMode = TRUE;
	s_motor[MOTOR_Z_NUM].timecnt = 0;

	Printf("MotorXYZ mode %d, [x]%d-[y]%d-[z]%d\n", mode, s_motor[MOTOR_X_NUM].targetStep, s_motor[MOTOR_Y_NUM].targetStep, s_motor[MOTOR_Z_NUM].targetStep);
	return DSCP_OK;
}

Bool Communication_MotorReset(Uint8 index)
{
	s_motor[index].status = MOTOR_TO_ZERO;
	Printf("Motor Reset\n");
	return DSCP_OK;
}

Bool Communication_MotorResetXYZ(void)
{
	s_motor[MOTOR_X_NUM].status = MOTOR_TO_WAIT_Z_RESET;
	s_motor[MOTOR_Y_NUM].status = MOTOR_TO_WAIT_Z_RESET;
	s_motor[MOTOR_Z_NUM].status = MOTOR_TO_ZERO;

	s_motor[MOTOR_X_NUM].targetStep = 0;
	s_motor[MOTOR_Y_NUM].targetStep = 0;
	s_motor[MOTOR_Z_NUM].targetStep = 0;

	s_motor[MOTOR_X_NUM].isXYZMode = TRUE;
	s_motor[MOTOR_Y_NUM].isXYZMode = TRUE;
	s_motor[MOTOR_Z_NUM].isXYZMode = TRUE;
	Printf("Motor XYZ Reset\n");
	return DSCP_OK;
}

Bool Communication_RequestStop(Uint8 index)
{
	s_motor[index].isRequestStop = TRUE;
	Printf("Motor Request Stop\n");
	return DSCP_OK;
}

Motor3DPos Communication_MotorGetXYZ(void)
{
	Motor3DPos pos;
	memset(&pos, 0, sizeof(Motor3DPos));
	pos.xpos = s_motor[MOTOR_X_NUM].currentSteps;
	pos.ypos = s_motor[MOTOR_Y_NUM].currentSteps;
	pos.zpos = s_motor[MOTOR_Z_NUM].currentSteps;
	Printf("Motor GetXYZ Pos\n");
	return pos;
}

Motor3DPos Communication_MotorGetXYZLimit(void)
{
	Motor3DPos pos;
	memset(&pos, 0, sizeof(Motor3DPos));
	pos.xpos = s_motor[MOTOR_X_NUM].maxSteps;
	pos.ypos = s_motor[MOTOR_Y_NUM].maxSteps;
	pos.zpos = s_motor[MOTOR_Z_NUM].maxSteps;
	Printf("Motor GetXYZ Limit Pos \n");
	return pos;
}

/*
 * Modbus检测
 *
 *  * */
static void Communication_ModbusTaskHandle(void *pvParameters)
{
	(void) pvParameters;
	Uint8 lastPost[5] = {0};
//	vTaskSuspend(NULL);
	while(1)
	{
		for(Uint8 i = 0;i<5;i++)
		{
			if(s_rxStatus[i] == Rx_Receiving)
			{
				if(s_recPost[i]>=7 && lastPost[i]==s_recPost[i])
				{
					if(i==RS485_INDEX_4) //注射器通信连接有干扰，暂时独立划分
					{
						memcpy(s_sy11Temp, s_sy11Rec, sizeof(s_sy11Rec));
					}
					else
					{
//						memcpy(s_recTemp, s_recResult, sizeof(s_recResult));
//						memset(s_recResult, 0, sizeof(s_recResult));
						memcpy(&s_recTemp[i][0], &s_recResult[i][0], MAX_DATA_LEN);
						memset(&s_recResult[i][0], 0, MAX_DATA_LEN);
					}
					s_recPost[i] = 0;
					s_rxStatus[i] = Rx_End;
				}
				lastPost[i] = s_recPost[i];
//					Printf("#%d %d#", lastPost[i],s_recPost[i]);
			}
			else if(s_rxStatus[i] == Rx_End)
			{
				s_rxStatus[i] = Rx_Idle;
			}
		}
		vTaskDelay(20 / portTICK_RATE_MS);
	}
}

/*
 * 监测
 *
 *  * */
static void Communication_MonitorTaskHandle(void *pvParameters)
{
	Uint16 DOStatus = 0;
//	vTaskSuspend(NULL);
	while(1)
	{
		for(Uint8 i = 0;i<3;i++)
		{
			DOStatus = 0;
			switch (s_motor[i].status)
			{
				case MOTOR_INIT:		//上电初始化并停止所有电机
					System_Delay(500);
					Communication_ServoOn(i);
					System_Delay(50);
					Communication_EmgsStop(i);
					System_Delay(50);
					Communication_SetHomeMode(i, HOMING_RESET_MODE_3);//设置复位模式为反转找ORGP信号
					System_Delay(50);
					Communication_SetORGP(i);//设置DI6为ORGP常开信号
					s_motor[i].status = MOTOR_IDLE;
					Printf("Stop %d Servo\n", i);
					break;
				case MOTOR_IDLE:		//空闲状态读取电机位置
					s_motor[i].currentSteps = Communication_GetAxisPosition(i);
					break;
				case MOTOR_TO_ZERO:		//复位状态发送复位指令并等待设备启动，不要一发送就马上读
					Communication_ResetAxis(s_motor[i].number);
					s_motor[i].status = MOTOR_TO_WAIT_ZERO;
					Printf("Reset %d Start\n", i);
					break;
				case MOTOR_TO_WAIT_ZERO:
					DOStatus = Communication_GetDOStatus(i);
					if((DOStatus&DO_STATUS_HOME)) //单坐标复位模式下上报事件
					{
						if(!s_motor[i].isXYZMode && s_motor[i].isSendEvent) //单轴运动上报结束事件
						{
							Communication_SendEvent(&s_motor[i], RESULT_FINISHED);
						}
						else if(s_motor[i].isXYZMode && s_motor[i].isSendEvent && s_motor[MOTOR_Z_NUM].status == MOTOR_IDLE)//多轴运动上报结束事件
						{
							if(s_motor[i].number == MOTOR_X_NUM && s_motor[MOTOR_Y_NUM].status == MOTOR_IDLE ) //Z轴复位完成后，XY都复位发送完成事件
							{
								Communication_SendEvent(&s_motor[i], RESULT_FINISHED);
								s_motor[MOTOR_X_NUM].isSendEvent = FALSE;
								s_motor[MOTOR_Y_NUM].isSendEvent = FALSE;
								s_motor[MOTOR_Z_NUM].isSendEvent = FALSE;
								Printf("Move X Send Event\n");
							}
							else if(s_motor[i].number == MOTOR_Y_NUM && s_motor[MOTOR_X_NUM].status == MOTOR_IDLE ) //Z轴复位完成后，XY都复位发送完成事件
							{
								Communication_SendEvent(&s_motor[i], RESULT_FINISHED);
								s_motor[MOTOR_X_NUM].isSendEvent = FALSE;
								s_motor[MOTOR_Y_NUM].isSendEvent = FALSE;
								s_motor[MOTOR_Z_NUM].isSendEvent = FALSE;
								Printf("Move Y Send Event\n");
							}
						}
						if(s_motor[i].isXYZMode && s_motor[i].number == MOTOR_Z_NUM)//多轴运动模式下Z轴复位完成
						{
							if(s_motor[MOTOR_X_NUM].targetStep > 0 && s_motor[MOTOR_Y_NUM].targetStep > 0) //非复位模式下，XY轴移动至目标位置
							{
								s_motor[MOTOR_X_NUM].status = MOTOR_SET_MOVE_MODE;
								s_motor[MOTOR_Y_NUM].status = MOTOR_SET_MOVE_MODE;
								s_motor[MOTOR_Z_NUM].status = MOTOR_TO_WAIT_XY_FINISH;
								s_motor[MOTOR_Z_NUM].timecnt = 0;
								Printf("MoveXY Start\n");
								Printf("Reset Z Done, Wait XY Finish\n");
							}
							else	//复位模式下，XY轴复位
							{
								s_motor[MOTOR_X_NUM].status = MOTOR_TO_ZERO;
								s_motor[MOTOR_Y_NUM].status = MOTOR_TO_ZERO;
								s_motor[MOTOR_Z_NUM].status = MOTOR_IDLE;
								s_motor[MOTOR_Z_NUM].timecnt = 0;
								Printf("MoveXY Reset\n");
								Printf("Reset Z Done, Wait XY Reset\n");
							}
						}
						else	//单轴模式下复位完成
						{
							s_motor[i].status = MOTOR_IDLE;
							Printf("Reset %d Done\n", i);
						}
					}
					break;
				case MOTOR_TO_STOP:
					DOStatus = Communication_GetDOStatus(i);
					if(DOStatus&DO_STATUS_BRKR)
					{
						if(!s_motor[i].isXYZMode && s_motor[i].isSendEvent)
						{
							Communication_SendEvent(&s_motor[i], RESULT_FINISHED);
						}
						s_motor[i].status = MOTOR_IDLE;
						Printf("Motor %d Stop Done\n", i);
					}
					if(s_motor[i].timeout < s_motor[i].timecnt++) //运行超时，全部停止
					{
						if(s_motor[i].isSendEvent)
						{
							Communication_SendEvent(&s_motor[i], RESULT_FAILED);
						}
						s_motor[MOTOR_X_NUM].status = MOTOR_IDLE;
						s_motor[MOTOR_Y_NUM].status = MOTOR_IDLE;
						s_motor[MOTOR_Z_NUM].status = MOTOR_IDLE;
						Communication_StopAllMotor();
						Printf("Motor Stop Timeout\n");
					}
					break;
				case MOTOR_SET_MOVE_MODE: //设置运动模式[绝对坐标/相对坐标/增量坐标][速度]
					Communication_TargetDefine(s_motor[i].number, s_motor[i].mode, s_speed);
					s_motor[i].status = MOTOR_SET_MOVE_POS;
					Printf("%d Move Mode [%d]\n", i, s_motor[i].mode);
					break;
				case MOTOR_SET_MOVE_POS: //设置运动步数
					Communication_SetTargetParam(s_motor[i].number, s_motor[i].targetStep);
					s_motor[i].status = MOTOR_SET_MOVE_START;
					Printf("%d Move Pram [%d]\n", i, s_motor[i].targetStep);
					break;
				case MOTOR_SET_MOVE_START: //位移启动
					Communication_MoveTarget(s_motor[i].number);
					s_motor[i].status = MOTOR_TO_TARGET_LOCATION;
					Printf("%d Move Start\n", i);
					break;
				case MOTOR_TO_TARGET_LOCATION: //检测位移完成信号
					DOStatus = Communication_GetDOStatus(i);
					if(DOStatus&DO_STATUS_TPOS)
					{
						if(!s_motor[i].isXYZMode && s_motor[i].isSendEvent)
						{
							Communication_SendEvent(&s_motor[i], RESULT_FINISHED);
						}
						else if(s_motor[i].isXYZMode && s_motor[i].isSendEvent && s_motor[i].number == MOTOR_Z_NUM)
						{
							Communication_SendEvent(&s_motor[i], RESULT_FINISHED);
						}
						s_motor[i].status = MOTOR_IDLE;
						Printf("Mov %d Done\n", i);
					}
					if(s_motor[i].timeout < s_motor[i].timecnt++) //运行超时，全部停止
					{
						if(s_motor[i].isSendEvent)
						{
							Communication_SendEvent(&s_motor[i], RESULT_FAILED);
						}
						s_motor[MOTOR_X_NUM].status = MOTOR_IDLE;
						s_motor[MOTOR_Y_NUM].status = MOTOR_IDLE;
						s_motor[MOTOR_Z_NUM].status = MOTOR_IDLE;
						Communication_StopAllMotor();
						Printf("Motor Wait XY Timeout\n");
					}
					break;
				case MOTOR_TO_WAIT_Z_RESET: //XY轴等待Z轴复位完成信号
					if(s_motor[MOTOR_Z_NUM].status == MOTOR_TO_WAIT_XY_FINISH)
					{
						DOStatus = Communication_GetDOStatus(MOTOR_Z_NUM);
					}
					if((DOStatus&DO_STATUS_HOME) && s_motor[i].status == MOTOR_TO_WAIT_XY_FINISH)
					{
						s_motor[MOTOR_X_NUM].status = MOTOR_SET_MOVE_MODE;
						s_motor[MOTOR_Y_NUM].status = MOTOR_SET_MOVE_MODE;
						s_motor[MOTOR_X_NUM].timecnt = 0;
						s_motor[MOTOR_Y_NUM].timecnt = 0;
						Printf("MoveXY Start\n");
					}
					if(s_motor[i].timeout < s_motor[i].timecnt++) //运行超时，全部停止
					{
						if(s_motor[i].isSendEvent)
						{
							Communication_SendEvent(&s_motor[i], RESULT_FAILED);
						}
						s_motor[MOTOR_X_NUM].status = MOTOR_IDLE;
						s_motor[MOTOR_Y_NUM].status = MOTOR_IDLE;
						s_motor[MOTOR_Z_NUM].status = MOTOR_IDLE;
						Communication_StopAllMotor();
						Printf("Motor Wait Z Reset Timeout\n");
					}
					break;
				case MOTOR_TO_WAIT_XY_FINISH: //Z轴等待XY轴完成信号
					if(s_motor[MOTOR_X_NUM].status == MOTOR_IDLE || s_motor[MOTOR_Y_NUM].status == MOTOR_IDLE)
					{
						DOStatus = Communication_GetDOStatus(MOTOR_X_NUM);
					}
					if(DOStatus&DO_STATUS_TPOS)
					{
						DOStatus = Communication_GetDOStatus(MOTOR_Y_NUM);
						if(DOStatus&DO_STATUS_TPOS)
						{
							s_motor[MOTOR_Z_NUM].status = MOTOR_SET_MOVE_MODE;
							Printf("MoveXY Finish, Z Mov Start\n");
						}
					}
					if(s_motor[i].timeout < s_motor[i].timecnt++) //运行超时，全部停止
					{
						if(s_motor[i].isSendEvent)
						{
							Communication_SendEvent(&s_motor[i], RESULT_FAILED);
						}
						s_motor[MOTOR_X_NUM].status = MOTOR_IDLE;
						s_motor[MOTOR_Y_NUM].status = MOTOR_IDLE;
						s_motor[MOTOR_Z_NUM].status = MOTOR_IDLE;
						Communication_StopAllMotor();
						Printf("Motor Wait XY Finish Timeout\n");
					}
					break;
				default:
					break;
			}
			if(s_motor[i].isSwitchToStart) //单轴移动启动
			{
				s_motor[i].status = MOTOR_SET_MOVE_MODE;
				s_motor[i].isSwitchToStart = FALSE;
			}
			if(s_motor[i].isRequestStop)
			{
				Communication_EmgsStop(s_motor[i].number);
				s_motor[i].status = MOTOR_TO_STOP;
				s_motor[i].isRequestStop = FALSE;
			}
//			Printf("Reset %d Start %d \n", i, s_motor[i].status);

			vTaskDelay(s_period / portTICK_RATE_MS);
		}
	}
}
