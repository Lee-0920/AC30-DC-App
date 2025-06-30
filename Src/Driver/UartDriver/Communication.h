#ifndef SRC_COMMUNICATION_H_
#define SRC_COMMUNICATION_H_

#include "stm32f4xx.h"
#include "Common/Types.h"

#define RS485_StopBits		 			  USART_StopBits_2
#define SY11_StopBits		 			  USART_StopBits_1
#define MOTOR_TOTAL		 			  		  3
#define MOTOR_X_NUM		 			  		  0
#define MOTOR_Y_NUM		 			  		  1
#define MOTOR_Z_NUM		 			  		  2
#define RS485_INDEX_1          0  //对应USART2
#define RS485_INDEX_2          1  //对应USART3
#define RS485_INDEX_3          2  //对应UART4
#define RS485_INDEX_4          3  //对应UART8
#define RS485_INDEX_5          4  //对应UART7

//RS485串口2配置 UART8
#define RS485_Tx1_PERIPHERAL_SYSCLK       45            //串口使用的时钟频率，APB1 为45M，APB2为90M
#define RS485_Tx1_IRQn                    UART8_IRQn
#define RS485_Tx1_CLK_CONFIG              RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8, ENABLE)
#define RS485_Tx1_RX_CONFIG               RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE)
#define RS485_Tx1_TX_CONFIG               RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE)
#define RS485_Tx1                         UART8
#define RS485_Tx1_GPIO_AF                 GPIO_AF_UART8
#define RS485_Tx1_TX_PIN                  GPIO_Pin_1
#define RS485_Tx1_TX_PinSource            GPIO_PinSource1
#define RS485_Tx1_TX_GPIO_PORT            GPIOE
#define RS485_Tx1_RX_PIN                  GPIO_Pin_0
#define RS485_Tx1_RX_PinSource            GPIO_PinSource0
#define RS485_Tx1_RX_GPIO_PORT            GPIOE
#define RS485_Tx1_UART_BAUD               9600

#define RS485_Tx1_DE_RE_SWITCH_PORT          GPIOB
#define RS485_Tx1_DE_RE_SWITCH_PIN           GPIO_Pin_9
#define RS485_Tx1_DE_RE_GPIO_CLK_CONFIG      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)

//RS485串口2配置 UART4
#define RS485_Tx2_PERIPHERAL_SYSCLK       45            //串口使用的时钟频率，APB1 为45M，APB2为90M
#define RS485_Tx2_IRQn                    UART4_IRQn
#define RS485_Tx2_CLK_CONFIG              RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE)
#define RS485_Tx2_RX_CONFIG               RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE)
#define RS485_Tx2_TX_CONFIG               RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE)
#define RS485_Tx2                         UART4
#define RS485_Tx2_GPIO_AF                 GPIO_AF_UART4
#define RS485_Tx2_TX_PIN                  GPIO_Pin_0
#define RS485_Tx2_TX_PinSource            GPIO_PinSource0
#define RS485_Tx2_TX_GPIO_PORT            GPIOA
#define RS485_Tx2_RX_PIN                  GPIO_Pin_1
#define RS485_Tx2_RX_PinSource            GPIO_PinSource1
#define RS485_Tx2_RX_GPIO_PORT            GPIOA
#define RS485_Tx2_UART_BAUD               9600

#define RS485_Tx2_DE_RE_SWITCH_PORT          GPIOA
#define RS485_Tx2_DE_RE_SWITCH_PIN           GPIO_Pin_4
#define RS485_Tx2_DE_RE_GPIO_CLK_CONFIG      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)

//RS485串口4配置 UART5
#define RS485_Tx3_PERIPHERAL_SYSCLK       45            //串口使用的时钟频率，APB1 为45M，APB2为90M
#define RS485_Tx3_IRQn                    UART5_IRQn
#define RS485_Tx3_CLK_CONFIG              RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE)
#define RS485_Tx3_RX_CONFIG               RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE)
#define RS485_Tx3_TX_CONFIG               RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)
#define RS485_Tx3                         UART5
#define RS485_Tx3_GPIO_AF                 GPIO_AF_UART5
#define RS485_Tx3_TX_PIN                  GPIO_Pin_12
#define RS485_Tx3_TX_PinSource            GPIO_PinSource12
#define RS485_Tx3_TX_GPIO_PORT            GPIOC
#define RS485_Tx3_RX_PIN                  GPIO_Pin_2
#define RS485_Tx3_RX_PinSource            GPIO_PinSource2
#define RS485_Tx3_RX_GPIO_PORT            GPIOD
#define RS485_Tx3_UART_BAUD               9600

#define RS485_Tx3_DE_RE_SWITCH_PORT          GPIOC
#define RS485_Tx3_DE_RE_SWITCH_PIN           GPIO_Pin_11
#define RS485_Tx3_DE_RE_GPIO_CLK_CONFIG      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)

//RS485串口4配置 USART2
#define RS485_Tx4_PERIPHERAL_SYSCLK       45            //串口使用的时钟频率，APB1 为45M，APB2为90M
#define RS485_Tx4_IRQn                    USART2_IRQn
#define RS485_Tx4_CLK_CONFIG              RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE)
#define RS485_Tx4_RX_CONFIG               RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)
#define RS485_Tx4_TX_CONFIG               RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)
#define RS485_Tx4                         USART2
#define RS485_Tx4_GPIO_AF                 GPIO_AF_USART2
#define RS485_Tx4_TX_PIN                  GPIO_Pin_2
#define RS485_Tx4_TX_PinSource            GPIO_PinSource2
#define RS485_Tx4_TX_GPIO_PORT            GPIOA
#define RS485_Tx4_RX_PIN                  GPIO_Pin_3
#define RS485_Tx4_RX_PinSource            GPIO_PinSource3
#define RS485_Tx4_RX_GPIO_PORT            GPIOA
#define RS485_Tx4_UART_BAUD               9600

#define RS485_Tx4_DE_RE_SWITCH_PORT          GPIOA
#define RS485_Tx4_DE_RE_SWITCH_PIN           GPIO_Pin_5
#define RS485_Tx4_DE_RE_GPIO_CLK_CONFIG      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)

//RS485串口5配置 UART7
#define RS485_Tx5_PERIPHERAL_SYSCLK       45            //串口使用的时钟频率，APB1 为45M，APB2为90M
#define RS485_Tx5_IRQn                    UART7_IRQn
#define RS485_Tx5_CLK_CONFIG              RCC_APB2PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE)
#define RS485_Tx5_RX_CONFIG               RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE)
#define RS485_Tx5_TX_CONFIG               RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE)
#define RS485_Tx5                         UART7
#define RS485_Tx5_GPIO_AF                 GPIO_AF_UART7
#define RS485_Tx5_TX_PIN                  GPIO_Pin_8
#define RS485_Tx5_TX_PinSource            GPIO_PinSource8
#define RS485_Tx5_TX_GPIO_PORT            GPIOE
#define RS485_Tx5_RX_PIN                  GPIO_Pin_7
#define RS485_Tx5_RX_PinSource            GPIO_PinSource7
#define RS485_Tx5_RX_GPIO_PORT            GPIOE
#define RS485_Tx5_UART_BAUD               38400

#define RS485_Tx5_DE_RE_SWITCH_PORT          GPIOE
#define RS485_Tx5_DE_RE_SWITCH_PIN           GPIO_Pin_9
#define RS485_Tx5_DE_RE_GPIO_CLK_CONFIG      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE)


/********通信地址**********/
#define MOBUDS_ADDR			 0x7F //伺服控制系统默认通讯地址

/********功能码**********/
#define FUNCTION_CODE_READ	 		 0x03
#define FUNCTION_CODE_WRITE_16BIT	 0x06
#define FUNCTION_CODE_WRITE_32BIT	 0x10

/********DO信号状态寄存器**********/
#define REG_DO_STATUS			 			0x005C		//UINT16 RW
/********辅助功能宏**********/
#define DO_STATUS_SRDY		 				(1<<0)		//伺服准备ok
#define DO_STATUS_SON		 				(1<<1)		//伺服启动
#define DO_STATUS_ZSPD		 				(1<<2)		//零速度检出
#define DO_STATUS_TSPD		 				(1<<3)		//目标速度到达
#define DO_STATUS_TPOS		 				(1<<4)		//目标位置到达
#define DO_STATUS_TQL		 				(1<<5)		//扭矩限制中
#define DO_STATUS_ALRM		 				(1<<6)		//伺服告警
#define DO_STATUS_BRKR		 				(1<<7)		//电磁抱闸
#define DO_STATUS_HOME		 				(1<<8)		//原点复位完成
#define DO_STATUS_OLW		 				(1<<9)		//电机过载
#define DO_STATUS_WARNY		 				(1<<10)		//其他告警

/********更新编码器绝对位置寄存器**********/
#define REG_UPDATE_ENCODER_ABS_P 			0x0062		//UINT16 RW
/********辅助功能宏**********/
#define UPDATE_ENCODER_NONE	 				(1<<0)		//无操作
#define UPDATE_ENCODER_UNCLEAR	 			(1<<1)		//更新编码器不清除状态
#define UPDATE_ENCODER_CLEAR				(1<<2)		//更新编码器并清除状态

/********绝对型坐标系统状态寄存器**********/
#define REG_ABS_AXIS_STATUS		 			0x0064		//UINT16 RW
/********辅助功能宏**********/
#define ABS_AXIS_OK			 				(0<<0)		//绝对位置正常
#define ABS_AXIS_LOST		 				(1<<0)		//绝对位置丢失
#define ABS_AXIS_VOLTAGE_OK	 				(0<<1)		//电池电压正常
#define ABS_AXIS_VOLTAGE_LOW 				(1<<1)		//电池电压低
#define ABS_AXIS_LOOP_OK	 				(0<<2)		//绝对圈数未溢出
#define ABS_AXIS_LOOP_OVER	 				(1<<2)		//绝对圈数溢出
#define ABS_AXIS_PUU_OK		 				(0<<3)		//PUU未溢位
#define ABS_AXIS_PUU_OVER	 				(1<<3)		//PUU溢位
#define ABS_AXIS_SET_DONE	 				(0<<4)		//绝对坐标建立完成
#define ABS_AXIS_SET_FAIL	 				(1<<4)		//绝对坐标未建立完成

/********其他寄存器地址**********/
#define REG_ENCODER_LOOP		 			0x0066		//INT16 RW 编码器绝对位置-圈数 -32767~+32767
#define REG_ENCODER_PULSE_PUU	 			0x0068		//INT32 RW 编码器绝对位置-脉冲orPUU (PULSE:0~16777216-1)(PUU:-2147483648~+2147483647)
#define REG_MODE_OPERATION		 			0x0102		//UINT16 RW 设置操作模式
#define REG_SPEED_TOLERANCE		 			0x0244		//UINT32 RW 速度误差告警条件，超过此设定值告警
#define REG_POSITION_TOLERANCE	 			0x0246		//UINT32 RW 位置误差告警条件，超过此设定值告警

/********特殊参数写入寄存器**********/
#define REG_SPACIAL_PARAM		 			0x0210		//UINT16 RW 特殊参数写入
/********特殊参数宏**********/
#define SPACIAL_RESET_PARAM	 				10			//P0~P7群参数重置(重置后需重新上电)
#define SPACIAL_ABS_SETTING	 				271			//先将 P2.069.(0x028A)设为1后并将伺服重上电以开启绝对型功能。
														//接着将P2.008(0x0210)设为 271，再将 P2.071(0x028E) 设为 0x0001，以建立絶对型原点坐标。

/********辅助功能寄存器**********/
#define REG_ASSISTED_FUNCTION 				0x023C		//UINT16
/********辅助功能宏**********/
#define ASSISTED_DEFAULT	 				0x00		//关闭所有功能
#define ASSISTED_FORCED_SERVO_ON			0x01		//强制启动Servo On
#define ASSISTED_EEPROM_DISABLE				0x05		//不写入EEPROM，增加寿命
#define ASSISTED_SIMULATION_MODE			0x06		//开启命令模拟
#define ASSISTED_BACKUP_EEPROM 				0x08		//备份所有参数到EEPROM,下次启动保持数值

/********绝对型编码器寄存器**********/
#define REG_ABS_ENCODER_SETTING				0x028A		//UINT16
/********编码器宏**********/
#define ENCODER_INC			 				0x00		//增量型操作
#define ENCODER_ABS			 				0x01		//绝对型操作
#define ENCODER_REJECT_PULSE				0x0000		//位置遗失不接受脉冲指令
#define ENCODER_ACCEPT_PULSE				0x0010		//位置遗失接受脉冲指令

/********讯息读取寄存器**********/
#define REG_MESSAGE							0x028C		//UINT16
/********讯息读取宏**********/
#define MESSAGE_PUU			 				0x00		//单位PUU
#define MESSAGE_PULSE		 				0x01		//单位PULSE
#define MESSAGE_ONLY_PUU	 				0x00		//编码器只能读到PUU(0x66无效，0x68puu)
#define MESSAGE_CNT_LOOP_PULSE 				0x02		//编码器可以读到计圈和脉冲数(0x66圈数，0x68脉冲)
#define MESSAGE_OVER_ALARM_DISABLE			0x00		//溢位不告警
#define MESSAGE_OVER_ALARM_ENABLE			0x04		//溢位告警

/********绝对位置归零寄存器**********/
#define REG_ABS_RESET						0x028E		//UINT16
/********绝对位置归零宏**********/
#define ABS_RESET_NONE			 			0x00
#define ABS_RESET_ON			 			0x01		//需P2.008(0x0210)设为271，且P2.069(0x028A)设为1才能启动

/********模式设定宏**********/
#define MODE_PT_INPUT		  	  			0x00 	    //位置模式(端子输入)
#define MODE_PR_CMD			  	  			0x01 	    //位置模式(通讯指令)
#define MODE_S	 	  						0x02 	    //速度模式(支持模拟量输入)
#define MODE_T		  						0x03 	    //扭矩模式(支持模拟量输入)
#define MODE_SZ		  	  					0x04 	    //速度模式(不支持模拟量)
#define MODE_TZ				  	  			0x05 	    //扭矩模式(不支持模拟量)
#define MODE_PT_S			  	  			0x06 	    //混合模式
#define MODE_PT_T			  	  			0x07 	    //双重模式
#define MODE_PR_S			  	  			0x08 	    //双重模式
#define MODE_PR_T			  	  			0x09 	    //双重模式
#define MODE_S_T			  	  			0x0A 	    //双重模式
#define MODE_PT_PR			  	  			0x0D 	    //双重模式
#define MODE_PT_PR_S		  	  			0x0E 	    //多重模式
#define MODE_PT_PR_T		  	  			0x0F 	    //多重模式
#define MODE_DIRECTION_CCW_CW  	  			0x0000 	    //方向:正转逆时针(CCW)，反转顺时针(CW)
#define MODE_DIRECTION_CW_CCW  	  			0x0100 	    //方向:正转顺时针(CW)，反转逆时针(CCW)
#define MODE_DIO_KEEP		  	  			0x0000 	    //模式切换时，DIO保持设定值，不会因模式切换而变更
#define MODE_DIO_RESET		  	  			0x1000 	    //模式切换时，DIO切换对应模式的默认值

/********数字输入DI寄存器地址**********/
#define REG_ADDR_BASE_DI	 				0x0214		//Uint16 数字输入DI1地址，每个地址+2 (DI1-DI8，禁止溢出)
#define REG_ADDR_BASE_DO	 				0x0224		//Uint16 数字输出DO1地址，每个地址+2 (DO1-DO8，禁止溢出)
#define ADDR_OFFSET			 				2			//地址长度 单位：字节

/********数字输入DI宏**********/
#define DI_SON				  	  			0x01 	    //伺服启动
#define DI_NL_CWL			  	  			0x22 	    //反向极限触点
#define DI_PL_CCWL			  	  			0x23 	    //正向极限触点
#define DI_ORGP				  	  			0x24 	    //原点触点(PR模式)
#define DI_STP			  	  				0x46 	    //电机停止

/********数字输出DO宏**********/
#define DO_B_CLOSE			 				0x0000		//常闭B接点
#define DO_A_OPEN			 				0x0100		//常开A接点
#define DO_SRDY				  	  			0x01 	    //上电后无异常，信号为On
#define DO_SON				  	  			0x02 	    //伺服启动无异常，信号为On
#define DO_HOME				  	  			0x09 	    //复位完成，启动复位命令后为Off,复位完成后保持On
#define DO_SNL				  	  			0x13 	    //软件负极限
#define DO_SPL				  	  			0x14 	    //软件正极限

/********通讯参数**********/
#define REG_MODBUS_DEVICE_ADDR				0x0300			//UINT16 默认0x007F, 输入范围0x0001~0x007F,地址为0xFF时广播,报文站号也是0xFF
#define REG_MODBUS_BAUD_RATE				0x0302			//UINT16 默认0x0033, 输入范围0x0000~0x0055,默认38400bps
#define REG_MODBUS_TYPE						0x0304			//UINT16 默认0x0066, 输入范围0x0000~0x0088,默认8,N,2(Modbus,RTU),两个停止位

/********Motion参数**********/
/********原点复位寄存器)**********/
#define REG_HOMING_MODE		 				0x0508		//UINT16 RW 原点复位模式(单位：PUU)
/********原点复位模式宏**********/
#define HOMING_LIMIT_ERROR		 			(0<<8)			//遇到极限时报错
#define HOMING_LIMIT_REV		 			(1<<8)			//遇到极限时反转
#define HOMING_Z_REV_FIND		 			(0<<4)			//返回找Z信号
#define HOMING_Z_FWD_FIND		 			(1<<4)			//不返回找Z信号or往前找Z信号
#define HOMING_Z_IGNORE		 				(2<<4)			//一律不找Z信号
#define HOMING_RESET_MODE_0	 				(0<<0)			//正转方向原点复位，正极限作为复位原点
#define HOMING_RESET_MODE_1	 				(1<<0)			//反转方向原点复位，反极限作为复位原点
#define HOMING_RESET_MODE_2	 				(2<<0)			//正转方向原点复归 ORG：OFF→ON 作为复归原点
#define HOMING_RESET_MODE_3	 				(3<<0)			//反转方向原点复归 ORG：OFF→ON 作为复归原点
#define HOMING_RESET_MODE_4	 				(4<<0)			//正转直接寻找Z 脉冲作为复归原点
#define HOMING_RESET_MODE_5	 				(5<<0)			//反转直接寻找Z 脉冲作为复归原点
#define HOMING_RESET_MODE_6	 				(6<<0)			//正转方向原点复归 ORG：ON→OFF 作为复归原点
#define HOMING_RESET_MODE_7	 				(7<<0)			//反转方向原点复归 ORG：ON→OFF 作为复归原点
#define HOMING_RESET_MODE_8	 				(8<<0)			//直接定义以目前位置当作原点
#define HOMING_RESET_MODE_9	 				(9<<0)			//正转方向扭力原点复归
#define HOMING_RESET_MODE_A 				(10<<0)			//反转方向扭力原点复归

#define REG_DI_STATUS	 					0x040E			//UINT16 R 数字输入DI1-DI6状态, 0x0010, 代表DI5为On
#define REG_HOMING_FIRST_SPEED	 			0x050A			//UINT32 RW 第一段高速复位速度,	默认值：1000, 范围：1~20000, 单位：0.1 rpm
#define REG_HOMING_SECOND_SPEED	 			0x050C			//UINT32 RW 第二段高速复位速度,	默认值：200, 范围：1~5000, 单位：0.1 rpm
#define REG_PR_CMD_TRIGGER		 			0x050E			//UINT16 RW PR命令触发器(触发内置程序) 输入0开始原点复位,输入2触发PR#2,以此类推
#define REG_SOFT_PL				 			0x0510			//INT32 RW 软件正极限,	默认值：2147483647, 范围：-2147483647~+2147483647, 单位：PPU			*
#define REG_SOFT_NL				 			0x0512			//INT32 RW 软件负极限,	默认值：2147483647, 范围：-2147483647~+2147483647, 单位：PPU			*
#define REG_AXIS_POSITION_PUU	 			0x0520			//INT32 RW 轴位置(PUU)(主编码器),	默认值：0, 范围：-2147483647~+2147483647, 单位：PPU		*
#define REG_AXIS_POSITION_PULSE	 			0x0524			//INT32 RW 轴位置(PULSE)(脉冲命令),	默认值：0, 范围：-2147483647~+2147483647, 单位：PULSE		*
#define REG_ACC_INC_DEC_0		 			0x0528			//INT16 RW 加/减速时间0,	默认值：200, 范围：1~65500, 单位：ms
#define REG_ACC_INC_DEC_1		 			0x052A			//INT16 RW 加/减速时间1,	默认值：300, 范围：1~65500, 单位：ms
#define REG_ACC_INC_DEC_2		 			0x052C			//INT16 RW 加/减速时间2,	默认值：500, 范围：1~65500, 单位：ms
#define REG_ACC_INC_DEC_3		 			0x052E			//INT16 RW 加/减速时间3,	默认值：600, 范围：1~65500, 单位：ms
#define REG_ACC_INC_DEC_4		 			0x0530			//INT16 RW 加/减速时间4,	默认值：800, 范围：1~65500, 单位：ms
#define REG_ACC_INC_DEC_5		 			0x0532			//INT16 RW 加/减速时间5,	默认值：900, 范围：1~65500, 单位：ms
#define REG_ACC_INC_DEC_6		 			0x0534			//INT16 RW 加/减速时间6,	默认值：1000, 范围：1~65500, 单位：ms
															//0-15 0x0528~0x0546

/********原点复位定义寄存器**********/
#define REG_HOMING_DEFINE		 			0x0600			//UINT32 RW 原点复归定义, 设定复归停止/复归速度/一段速度/二段速度/上电是否复位
/********原点复位定义宏**********/
#define HOMING_DEFINE_FINISH_STOP 			(0<<0)			//复位完成后停止
#define HOMING_DEFINE_ACC_0					(0<<8)			//加速度, 	 默认最低加速度 可设置15段速度 P5.020~P5.035
#define HOMING_DEFINE_DEC1_0				(0<<12)			//第一段减速, 默认最低速度 可设置15段速度 P5.020~P5.035
#define HOMING_DEFINE_DEC2_0				(0<<16)			//第二段减速, 默认最低速度 可设置15段速度 P5.020~P5.035
#define HOMING_DEFINE_DLY_0					(0<<24)			//延迟时间,   默认最低时间 可设置15段速度 P5.040~P5.055
#define HOMING_DEFINE_BOOT_OFF				(0<<28)			//上电不做原点复位
#define HOMING_DEFINE_BOOT_ON				(1<<28)			//上电做原点复位(第一次Servo On)

/********路径1属性定义寄存器**********/
#define REG_PATH_1_DEFINE		 			0x0604			//UINT32 RW 原点复归定义, 设定复归停止/复归速度/一段速度/二段速度/上电是否复位
/********路径1属性定义宏**********/
#define PATH_SINGLE				 			(2<<0)			//完成后停止
#define PATH_AUTO				 			(3<<0)			//自动加载下一路径停止
#define PATH_OPT_ABS			 			(0<<4)			//绝对寻址
#define PATH_OPT_REL			 			(1<<4)			//相对寻址
#define PATH_OPT_INC			 			(2<<4)			//增量寻址
#define PATH_ACC_0				 			(0<<8)			//加速度 200ms 0-15
#define PATH_ACC_1				 			(1<<8)			//加速度 300ms
#define PATH_ACC_2				 			(2<<8)			//加速度 500ms
#define PATH_DEC_0				 			(0<<12)			//减速度 200ms 0-15
#define PATH_DEC_1				 			(1<<12)			//减速度 300ms
#define PATH_DEC_2				 			(2<<12)			//减速度 500ms
#define PATH_SPD_0				 			(0<<16)			//速度 200 单位0.1rpm
#define PATH_SPD_1				 			(1<<16)			//速度 500 单位0.1rpm
#define PATH_SPD_2				 			(2<<16)			//速度 1000 单位0.1rpm
#define PATH_DLY_0				 			(0<<24)			//延迟时间

/********路径1数据寄存器**********/
#define REG_PATH_1_DATA		 			0x0606			//INT32 RW 路径目标值

/********命令触发宏**********/
#define PR_CMD_TRIGGER_STOP	 				0x03E8			//紧急停止
#define PR_CMD_TRIGGER_RESET	 			0x0000			//原点复位
#define PR_CMD_TRIGGER_PATH_1	 			0x0001			//执行路径1

/**
 * @brief 步进电机运行结果
 */
typedef enum
{
    RESULT_FINISHED,                // 操作正常完成。
    RESULT_FAILED,                  // 操作中途出现故障，未能完成。
    RESULT_STOPPED,                 // 操作被停止。
    RESULT_COLLISION_0,          // 针碰撞
    RESULT_COLLISION_1,         // 针碰撞
    RESULT_MOVE_OUT_SENSOR_FAIL_X,  //X轴移不出传感器
    RESULT_MOVE_OUT_SENSOR_FAIL_Z,  //Z轴移不出传感器
    RESULT_MOVE_IN_SENSOR_FAIL_X,    //X轴找不到传感器
    RESULT_MOVE_IN_SENSOR_FAIL_Z,    //Z轴找不到传感器
    RESULT_MOVE_OUT_SENSOR_FAIL_SYR,  //注射器移不出传感器,
    RESULT_MOVE_IN_SENSOR_FAIL_SYR,    //注射器找不到传感器
    RESULT_DRIVER_ERROR,            //驱动错误
    MAX_MOVE_RESULT
}MoveResult;

typedef enum
{
    MOTOR_MOVE_ABSOLUTE_MODE,//绝对模式
    MOTOR_MOVE_RELATIVE_MODE,//相对模式
    MOTOR_MOVE_INCREATE_MODE,
    MAX_MOTOR_MOVE_MODE,
}MotorMode;

typedef enum
{
    MOTOR_IDLE,
	MOTOR_INIT,
    MOTOR_TO_WAIT_Z_RESET,
	MOTOR_TO_WAIT_XY_FINISH,
    MOTOR_TO_TARGET_LOCATION,
    MOTOR_TO_ZERO,
	MOTOR_TO_WAIT_ZERO,
	MOTOR_TO_STOP,
	MOTOR_SET_MOVE_MODE,
	MOTOR_SET_MOVE_POS,
	MOTOR_SET_MOVE_START,
}MotorStatus;

typedef enum
{
    FORWARD, //向前
    BACKWARD, //向后
    MAX
}XZDirection;

typedef struct
{
    Int32 xpos;//x位置
    Int32 ypos;//x位置
    Int32 zpos;//x位置
}Motor3DPos;

//上电状态
typedef enum
{
	Rx_Idle,
	Rx_Receiving,
	Rx_End,
}RxStatus;

typedef struct
{
	Uint8 number;
    Int32 currentSteps;//当前位置
    Uint32 maxSteps;//单次启动的最大运行步数
    __IO Bool isRequestStop;
    Bool isSendEvent;    
	Bool isSwitchToStart;
	Bool isXYZMode;
    MotorStatus status;
    MotorMode mode;
    XZDirection dir;
    Int32 targetStep;
    Bool isUseDefaultParam;
    MoveResult moveResult;
    Uint16 timeout;
    Uint16 timecnt;
}Motor;

void Communication_Init(void);
void Communication_Send(Uint8 index, char* Data, int len);
void Communication_Read(Uint8 index, char* Data, int len);
void Communication_Clear(Uint8 index);
Uint16 Communication_CRC(unsigned char *buffer,unsigned len);
void Communication_SendEventOpen(Uint8 index);
void Communication_ServoOn(Uint8 index);
void Communication_ServoOff(Uint8 index);
void Communication_SetORGP(Uint8 index);
void Communication_SetHomeMode(Uint8 index, Uint8 mode);
void Communication_HomingDefine(Uint8 index);
Bool Communication_ResetAxis(Uint8 index);
void Communication_EmgsStop(Uint8 index);
void Communication_MoveTarget(Uint8 index);
void Communication_TargetDefine(Uint8 index, Uint8 mode, Uint8 speed);
void Communication_SetTargetParam(Uint8 index, Int32 param);
void Communication_SetPRMode(Uint8 index);
Int32 Communication_GetAxisPosition(Uint8 index);
void Communication_ClearAxisPosition(Uint8 index);
void Communication_SetDIFunction(Uint8 index, Uint16 pram);
Uint16 Communication_GetDIFunction(Uint8 index);
Uint16 Communication_GetDIStatus(Uint8 index);
Uint16 Communication_GetDOStatus(Uint8 index);
void Communication_DOReport(Uint16 arg);
Bool Communication_MotorStart(Uint8 index, Uint8 mode, Int32 steps);
Bool Communication_MotorStartXYZ(Uint8 mode, Motor3DPos mulpos);
Bool Communication_MotorReset(Uint8 index);
Bool Communication_MotorResetXYZ(void);
Motor3DPos Communication_MotorGetXYZ(void);
Motor3DPos Communication_MotorGetXYZLimit(void);
Bool Communication_RequestStop(Uint8 index);
void Communication_SetHomingFirstSpeed(Uint8 index, Uint32 speed);
void Communication_SetHomingSecondSpeed(Uint8 index, Uint32 speed);
void Communication_SaveToEPPROM(Uint8 index);
void Communication_SetRunSpeed(Uint8 speed);
Uint8 Communication_GetRunSpeed(void);

static inline void Communication_Write(Uint8 index, unsigned char data)
{
	if(RS485_INDEX_1 == index)
	{
		USART_SendData(RS485_Tx1, data);
	}
	else if(RS485_INDEX_2 == index)
	{
		USART_SendData(RS485_Tx2, data);
	}
	else if(RS485_INDEX_3 == index)
	{
		USART_SendData(RS485_Tx3, data);
	}
	else if(RS485_INDEX_4 == index)
	{
		USART_SendData(RS485_Tx4, data);
	}
	else
	{
		USART_SendData(RS485_Tx5, data);
	}
}

static inline bool Communication_IsTxInterruptTriggered(Uint8 index)
{
	if(RS485_INDEX_1 == index)
	{
		return (USART_GetITStatus(RS485_Tx1, USART_IT_TXE));
	}
	else if(RS485_INDEX_2 == index)
	{
		return (USART_GetITStatus(RS485_Tx2, USART_IT_TXE));
	}
	else if(RS485_INDEX_3 == index)
	{
		return (USART_GetITStatus(RS485_Tx3, USART_IT_TXE));
	}
	else if(RS485_INDEX_4 == index)
	{
		return (USART_GetITStatus(RS485_Tx4, USART_IT_TXE));
	}
	else
	{
		return (USART_GetITStatus(RS485_Tx5, USART_IT_TXE));
	}
}

static inline bool Communication_IsRxInterruptTriggered(Uint8 index)
{
    if(RS485_INDEX_1 == index)
	{
		return (USART_GetITStatus(RS485_Tx1, USART_IT_RXNE));
	}
	else if(RS485_INDEX_2 == index)
	{
		return (USART_GetITStatus(RS485_Tx2, USART_IT_RXNE));
	}
	else if(RS485_INDEX_3 == index)
	{
		return (USART_GetITStatus(RS485_Tx3, USART_IT_RXNE));
	}
	else if(RS485_INDEX_4 == index)
	{
		return (USART_GetITStatus(RS485_Tx4, USART_IT_RXNE));
	}
	else
	{
		return (USART_GetITStatus(RS485_Tx5, USART_IT_RXNE));
	}
}

static inline bool Communication_IsTxInterruptEnabled(Uint8 index)
{
    if(RS485_INDEX_1 == index)
   	{
   		return ((RS485_Tx1->CR1) & USART_CR1_RXNEIE);
   	}
   	else if(RS485_INDEX_2 == index)
   	{
   		return ((RS485_Tx2->CR1) & USART_CR1_RXNEIE);
   	}
   	else if(RS485_INDEX_3 == index)
   	{
   		return ((RS485_Tx3->CR1) & USART_CR1_RXNEIE);
   	}
   	else if(RS485_INDEX_4 == index)
	{
		return ((RS485_Tx4->CR1) & USART_CR1_RXNEIE);
	}
   	else
   	{
   		return ((RS485_Tx5->CR1) & USART_CR1_RXNEIE);
   	}
}

static inline void Communication_EnableTxInterrupt(Uint8 index)
{
    if(RS485_INDEX_1 == index)
	{
    	USART_ITConfig(RS485_Tx1, USART_IT_TXE, ENABLE);
	}
	else if(RS485_INDEX_2 == index)
	{
		USART_ITConfig(RS485_Tx2, USART_IT_TXE, ENABLE);
	}
	else if(RS485_INDEX_3 == index)
	{
		USART_ITConfig(RS485_Tx3, USART_IT_TXE, ENABLE);
	}
	else if(RS485_INDEX_4 == index)
	{
		USART_ITConfig(RS485_Tx4, USART_IT_TXE, ENABLE);
	}
	else
	{
		USART_ITConfig(RS485_Tx5, USART_IT_TXE, ENABLE);
	}
}

static inline void Communication_DisableTxInterrupt(Uint8 index)
{
    if(RS485_INDEX_1 == index)
   	{
    	USART_ITConfig(RS485_Tx1, USART_IT_TXE, DISABLE);
   	}
   	else if(RS485_INDEX_2 == index)
   	{
   		USART_ITConfig(RS485_Tx2, USART_IT_TXE, DISABLE);
   	}
   	else if(RS485_INDEX_3 == index)
   	{
   		USART_ITConfig(RS485_Tx3, USART_IT_TXE, DISABLE);
   	}
   	else if(RS485_INDEX_4 == index)
	{
		USART_ITConfig(RS485_Tx4, USART_IT_TXE, DISABLE);
	}
   	else
   	{
   		USART_ITConfig(RS485_Tx5, USART_IT_TXE, DISABLE);
   	}
}

static void Communication_RS485_SwitchToTx(Uint8 index)
{
    if(RS485_INDEX_1 == index)
	{
    	RS485_Tx1_DE_RE_SWITCH_PORT->BSRRL |= RS485_Tx1_DE_RE_SWITCH_PIN;
	}
	else if(RS485_INDEX_2 == index)
	{
		RS485_Tx2_DE_RE_SWITCH_PORT->BSRRL |= RS485_Tx2_DE_RE_SWITCH_PIN;
	}
	else if(RS485_INDEX_3 == index)
	{
		RS485_Tx3_DE_RE_SWITCH_PORT->BSRRL |= RS485_Tx3_DE_RE_SWITCH_PIN;
	}
	else if(RS485_INDEX_4 == index)
	{
		RS485_Tx4_DE_RE_SWITCH_PORT->BSRRL |= RS485_Tx4_DE_RE_SWITCH_PIN;
	}
	else
	{
		RS485_Tx5_DE_RE_SWITCH_PORT->BSRRL |= RS485_Tx5_DE_RE_SWITCH_PIN;
	}
}

static inline void Communication_RS485_SwitchToRx(Uint8 index)
{
    if(RS485_INDEX_1 == index)
   	{
    	RS485_Tx1_DE_RE_SWITCH_PORT->BSRRH |= RS485_Tx1_DE_RE_SWITCH_PIN;
   	}
   	else if(RS485_INDEX_2 == index)
   	{
   		RS485_Tx2_DE_RE_SWITCH_PORT->BSRRH |= RS485_Tx2_DE_RE_SWITCH_PIN;
   	}
   	else if(RS485_INDEX_3 == index)
   	{
   		RS485_Tx3_DE_RE_SWITCH_PORT->BSRRH |= RS485_Tx3_DE_RE_SWITCH_PIN;
   	}
   	else if(RS485_INDEX_4 == index)
	{
		RS485_Tx4_DE_RE_SWITCH_PORT->BSRRH |= RS485_Tx4_DE_RE_SWITCH_PIN;
	}
   	else
   	{
   		RS485_Tx5_DE_RE_SWITCH_PORT->BSRRH |= RS485_Tx5_DE_RE_SWITCH_PIN;
   	}
}
#endif
