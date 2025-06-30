/**
 * @addtogroup module_CmdLine
 * @{
 */

/**
 * @file
 * @brief 应用接口：命令行实现。
 * @details 定义各应用命令及其处理函数。
 * @version 1.0.0
 * @author kim.xiejinqiang
 * @date 2012-5-21
 */


#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdlib.h>
#include "Tracer/Trace.h"
#include "Console/Console.h"
#include "Driver/System.h"
#include "CmdLine.h"
#include "SystemConfig.h"
#include "DeviceIndicatorLED.h"
#include "UpdateDriver.h"
#include "DeviceUpdate/UpdateHandle.h"
#include "McuFlash.h"
#include "Driver/HardwareType.h"
#include "Manufacture/VersionInfo.h"
#include "SolenoidValve/ValveManager.h"

#include "CheckLeaking/CheckLeakingControl.h"
#include "Driver/LiquidDriver/RelayMap.h"
#include "Peripheral/DetectSensorManager.h"
#include "Driver/UartDriver/Communication.h"
#include "PeristalticPump/SyringeManager.h"
#include <TempDriver/EnvTempCollect.h>
#include <TempDriver/TempADCollect.h>
#include "Driver/LiquidDriver/AD5175Driver.h"

// 命令行版本定义，命令有变更时，需要相应更新本版本号
const CmdLineVersion g_kCmdLineVersion =
{
        1,      // 主版本号
        0,      // 次版本号
        0,      // 修订版本号
        0       // 编译版本号
};
static  Uint8 s_currPumpNum = 0;

// 命令处理函数声明列表
static int Cmd_help(int argc, char *argv[]);
static int Cmd_welcome(int argc, char *argv[]);
static int Cmd_version(int argc, char *argv[]);
static int Cmd_showparam(int argc, char *argv[]);
static int Cmd_reset(int argc, char *argv[]);
static int Cmd_trace(int argc, char *argv[]);
static int Cmd_demo(int argc, char *argv[]);

//系统命令
static int Cmd_flash(int argc, char *argv[]);
static int Cmd_RestoreInit(int argc, char *argv[]);
static int Cmd_IAP(int argc, char *argv[]);
static int Cmd_SetBlink(int argc, char *argv[]);
static int Cmd_TaskState(int argc, char *argv[]);
static int Cmd_Hardware(int argc, char *argv[]);

//阀命令函数
static int Cmd_valve(int argc, char *argv[]);

//漏液检测命令
static int Cmd_ChcekLeaking(int argc, char *argv[]);

//继电器测试命令
static int Cmd_Relay(int argc, char *argv[]);

//传感器命令
static int Cmd_Sensor(int argc, char *argv[]);

//伺服驱动器命令
static int Cmd_Debug(int argc, char *argv[]);

//注射器命令
static int Cmd_SY11(int argc, char *argv[]);

//温控命令
static int Cmd_Temp(int argc, char *argv[]);

//AD5175命令
static int Cmd_5175(int argc, char *argv[]);

static void InfoOutput(void *argument);
/**
 * @brief 命令行命令表，保存命令关键字与其处理函数之间的对应关系。
 * @details 每条命令对应一个结点，结点包括：命令关键字、处理函数、简短描述文本。
 */
const CmdLineEntry g_kConsoleCmdTable[] =
{
    { "demo",       Cmd_demo,       "\t\t: Demo for cmd implementation and param parse" },
    { "trace",      Cmd_trace,      "\t\t: Trace level" },
    { "welcome",    Cmd_welcome,    "\t\t: Display welcome message" },
    { "version",    Cmd_version,    "\t\t: Display version infomation about this application" },
    { "reset",      Cmd_reset,      "\t\t: Reset system" },
    { "help",       Cmd_help,       "\t\t: Display list of commands. Short format: h or ?" },

    { "flash",      Cmd_flash,      "\t\t: Flash read write erase operation." },
    { "RI",         Cmd_RestoreInit, "\t\t:Restore system initial state." },
    { "iap",        Cmd_IAP,         "\t\t:Provides erase and jump operations." },
    { "blink",      Cmd_SetBlink,    "\t\t:Set the duration of equipment indicator, on time and off time.Uint milliseconds." },
    { "taskstate",  Cmd_TaskState,  "\t\t: Out put system task state." },
    { "hardware",   Cmd_Hardware,  "\t\t: Read Hardware Info." },

    { "valve",      Cmd_valve,      "\t\t: valve set read operation." },
	{ "check",      Cmd_ChcekLeaking,       "\t\t: start |stop | set the check leaking period " },
	{ "relay",      Cmd_Relay,   "\t\t: relay control " },
	{ "sensor",     Cmd_Sensor,   "\t\t: sensor detect " },
	{ "s",      	Cmd_Debug,  "\t\t: servo driver control" },
	{ "sy",      	Cmd_SY11,  "\t\t: SY11 control" },
	{ "temp",       Cmd_Temp,       "\t\t: get temperature" },
	{ "ad",         Cmd_5175,       "\t\t: set ad5175 param" },
    { "?",          Cmd_help,       0 },
    { "h",          Cmd_help,       0 },
    { "showparam",  Cmd_showparam,  0 },
    { 0, 0, 0 }
};


/**
 * @brief 判断第一个字串等于第二个字串。
 * @details 本函数与strcmp相比，预先做了有效性检查。
 * @param str1 要比较的字串1。
 * @param str2 要比较的字串2，不能为NULL。
 * @return 是否相等。
 */
Bool IsEqual(const char* str1, const char* str2)
{
    return (0 == strcmp(str1, str2)) ? TRUE : FALSE;
}

static xTaskHandle s_InfoOutputHandle;
static Uint16 s_getInfoTime = 0;
typedef enum
{
    LC,
    OA,
    TC,
    SYS,
}InfoOutputMode;

static InfoOutputMode s_InfoOutputMode = LC;

void CmdLine_Init(void)
{
    xTaskCreate(InfoOutput, "InfoOutput", CMDCLINE_INFOOUTPUT_STK_SIZE, NULL,
            CMDCLINE_INFOOUTPUT_TASK_PRIO, &s_InfoOutputHandle);
}

static void InfoOutput(void *argument)
{
    (void) argument;
    vTaskSuspend(NULL);
    while (1)
    {
        vTaskDelay(s_getInfoTime / portTICK_RATE_MS);
        switch(s_InfoOutputMode)
        {
            case LC:
//            	Communication_MoveTarget(2);
            	break;
            case OA:
                break;
            case SYS:
                System_TaskStatePrintf();
//                System_StateTimerValue();
                break;
        }
//        for(Uint8 i = 0;i < 3;i++)
//		{
//        	Int32 pos = Communication_GetAxisPosition(i);
//        	System_Delay(5);
//			Printf("Axis [%d] Pos %d\n", i, pos);
//		}
        Motor3DPos pos = Communication_MotorGetXYZ();
		Printf("Axis XYZ Pos [%d][%d][%d]\n", pos.xpos, pos.ypos, pos.zpos);
    }
}

//*****************************************************************************
//
// 系统常规命令处理函数
//
//*****************************************************************************
#include "console/port/driver/ConsoleUart.h"

int Cmd_TaskState(int argc, char *argv[])
{
    if(IsEqual(argv[1], "start"))
    {
        if (argv[2] && atoi(argv[2]) >= 10)
        {
            s_getInfoTime = atoi(argv[2]);
            s_InfoOutputMode = SYS;
            vTaskResume(s_InfoOutputHandle);
        }
        else
        {
            Printf("Invalid param %s\n", argv[2]);
        }
    }
    else if(IsEqual(argv[1], "stop"))
    {
        vTaskSuspend(s_InfoOutputHandle);
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== taskstate commands ======\n");
        Printf(" taskstate start [TIME]:ms\n");
        Printf(" taskstate stop        :\n");
    }
    return (0);
}

int Cmd_Hardware(int argc, char *argv[])
{
    Uint8 type = 0;
    if (IsEqual(argv[1], "type"))
    {
        type = HardwareType_GetValue();
        Printf("Hardware Type: %d\n", type);
    }
    else
    {
    	type = HardwareType_GetValue();
    }
    return 0;
}

int Cmd_SetBlink(int argc, char *argv[])
{
    if (argv[1])
    {
        if (argv[2] && argv[3])
        {
            DeviceIndicatorLED_SetBlink(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== blink commands ======\n");
        Printf(" blink [DURATION] [ONTIME] [OFFTIME]:\n");
    }

    return (0);
}

int Cmd_IAP(int argc, char *argv[])
{
    if (IsEqual(argv[1], "erase"))
    {
        UpdateHandle_StartErase();
    }
    else if (IsEqual(argv[1], "write"))
    {
        if (argv[2] && argv[3] && argv[4])
        {
            UpdateHandle_WriteProgram((u8 *)argv[2], atoi(argv[3]), atoi(argv[4]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "check"))
    {
        if (argv[2])
        {
            UpdateHandle_CheckIntegrity(atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "read"))
    {
        if (argv[2] && argv[3])
        {
            uint8_t str[30]={""};
            UpdateDriver_Read(atoi(argv[2]),atoi(argv[3]),str);
            Printf("\n");
            for(int i = 0; i< atoi(argv[3]); i++)
            {
                Printf("0x%02x ",str[i]);
            }
            Printf("\n%s",str);
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "getmax"))
    {
        Printf("%d\n", UpdateHandle_GetMaxFragmentSize());
    }
    else if (IsEqual(argv[1], "getmode"))
    {
        DeviceRunMode mode = UpdateHandle_GetRunMode();
        Printf("%d\n", mode);
    }
#ifdef _CS_APP
    else if (IsEqual(argv[1], "updater"))
    {
        UpdateHandle_EnterUpdater();
    }
#else
    else if (IsEqual(argv[1], "app"))
    {
        UpdateHandle_EnterApplication();
    }
#endif
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== iap commands ======\n");
        Printf(" iap erase: \n");
        Printf(" iap write [TEXT]   [NUM]  [SEQ]: \n");
        Printf(" iap check [CRC16]              : \n");
        Printf(" iap read  [OFFSET] [NUM]       : \n");
#ifdef _CS_APP
        Printf(" iap updater                    : \n");
#else
        Printf(" iap app                        : \n");
#endif
        Printf(" iap getmax                     : \n");
        Printf(" iap getmode                    : \n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}
   
int Cmd_flash(int argc, char *argv[])
{
    if (IsEqual(argv[1], "deletewrite"))//不保留原始数据的写
    {
        if (argv[2] && argv[3] && argv[4])
        {
            McuFlash_DeleteWrite(atoi(argv[2]), atoi(argv[3]),(u8 *)argv[4]);
            Printf("\nWriteAddr 0x%x ",atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "write"))//保留原始数据的写
    {
        if (argv[2] && argv[3] && argv[4])
        {
            McuFlash_Write(atoi(argv[2]), atoi(argv[3]),(u8 *)argv[4]);
            Printf("\nWriteAddr 0x%x ",atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "read"))//读FLASH数据
    {
        if (argv[2] && argv[3])
        {
            uint8_t str[30]={""};
            McuFlash_Read(atoi(argv[2]),atoi(argv[3]),str);
            Printf("\nReadAddr 0x%x\n",atoi(argv[2]));
            for(int i = 0; i< atoi(argv[3]); i++)
            {
                Printf("0x%02x ",str[i]);
            }
            Printf("\n%s",str);
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "erase"))//擦除
    {
        if (argv[2])
        {
           McuFlash_EraseSector(atoi(argv[2]));
           Printf("\nEraseAddr 0x%x", atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== flash commands ======\n");
        Printf(" flash deletewrite [ADDR] [NUM] [TEXT]: \n");
        Printf(" flash write       [ADDR] [NUM] [TEXT]: \n");
        Printf(" flash read        [ADDR] [NUM]       : \n");
        Printf(" flash erase       [ADDR]             : \n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

int Cmd_RestoreInit(int argc, char *argv[])
{
    //以下操作可能会因为没有启动而控制台出现错误提醒。
	ValveManager_SetValvesMap(0);//这只保证定量运行的时候不会关闭所有的阀。
    return 0;
}


//*****************************************************************************
//
// 阀命令处理函数
//
//*****************************************************************************
int Cmd_valve(int argc, char *argv[])
{
    if (IsEqual(argv[1], "map"))
    {
        if (argv[2] )
        {
            ValveManager_SetValvesMap(atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param");
        }
    }
    else if (IsEqual(argv[1], "open"))
    {
        if (argv[2] && atoi(argv[2]) > 0)
        {
            ValveManager_SetValvesMap((Uint32)( 1 << (atoi(argv[2]) - 1)));
        }
        else
        {
            Printf("Invalid param");
        }
    }
    else if (IsEqual(argv[1], "closeall"))
    {
        ValveManager_SetValvesMap(0);
    }
    else if (IsEqual(argv[1], "get"))
    {
        Uint32 map = ValveManager_GetValvesMap();
        Printf("ValvesMap: 0x%4x\n", map);
    }
    else if (IsEqual(argv[1], "total"))
    {
        Printf("total: %d\n", ValveManager_GetTotalValves());
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== valve commands ======\n");
        Printf(" valve map  [MAP]: Set the map of the valve. map must be 0x0 - 0x%x.\n\n", SOLENOIDVALVE_MAX_MAP);
        Printf(" valve open [NUM]: Open valve NUM.Num must be 1 - %d.\n", ValveManager_GetTotalValves());
        Printf(" valve closeall  : Close all valves. \n");
        Printf(" valve get       : Mapping map of the solenoid valve. \n");
        Printf(" valve total     : Total number of solenoid valves. \n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

//*****************************************************************************
//
// 命令处理函数
//
//*****************************************************************************
// 显示帮助，简单显示命令列表
int Cmd_help(int argc, char *argv[])
{
    CmdLineEntry *cmdEntry;

    ConsoleOut("\nAvailable commands\n");
    ConsoleOut("------------------\n");

    cmdEntry = (CmdLineEntry *) &g_kConsoleCmdTable[0];

    // 遍历整个命令表，打印出有提示信息的命令
    while (cmdEntry->cmdKeyword)
    {
        // 延时一下，等待控制台缓冲区空出足够的空间
        System_Delay(10);

        if (cmdEntry->cmdHelp)
            ConsoleOut("%s%s\n", cmdEntry->cmdKeyword, cmdEntry->cmdHelp);

        cmdEntry++;
    }

    return (0);
}

int Cmd_version(int argc, char *argv[])
{
    ManufVersion softVer = VersionInfo_GetSoftwareVersion();
    ManufVersion hardVer = VersionInfo_GetHardwareVersion();
    ConsoleOut("Cmd Ver: %d.%d.%d.%d\n", g_kCmdLineVersion.major, g_kCmdLineVersion.minor, g_kCmdLineVersion.revision, g_kCmdLineVersion.build);
    ConsoleOut("Soft Ver: %d.%d.%d.%d\n", softVer.major, softVer.minor, softVer.revision, softVer.build);
    ConsoleOut("Pcb Ver: %d.%d.%d.%d\n", hardVer.major, hardVer.minor, hardVer.revision, hardVer.build);

    return(0);
}

int Cmd_welcome(int argc, char *argv[])
{
    Console_Welcome();
    return(0);
}


// 显示参数
int Cmd_showparam(int argc, char *argv[])
{
    int i = 0;

    ConsoleOut("The params is:\n");
    for (i = 1; i < argc; i++)
    {
        ConsoleOut("    Param %d: %s\n", i, argv[i]);
    }

    return(0);
}


int Cmd_reset(int argc, char *argv[])
{
    Printf("\n\n\n");
    System_Delay(10);
    System_Reset();
    return (0);
}

int Cmd_trace(int argc, char *argv[])
{
    if (argv[1])
    {
        Trace_SetLevel(atoi(argv[1]));
    }
    else
    {
        Printf("L: %d\n", Trace_GetLevel());
    }

    return (0);
}

// 命令处理函数示例
int Cmd_demo(int argc, char *argv[])
{
    if (IsEqual(argv[1], "subcmd1"))
    {
        if (IsEqual(argv[2], "param"))
        {
            // 调用相关功能函数
            Printf("Exc: subcmd1 param");
        }
    }
    else if (IsEqual(argv[1], "subcmd2"))
    {
        Printf("Exc: subcmd2");
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== Sub commands ======\n");
        Printf(" mycmd subcmd1 param : Sub command description\n");
        Printf(" mycmd subcmd2       : Sub command description\n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }

    return (0);
}

int Cmd_MCP4651(int argc, char *argv[])
{
	if (IsEqual(argv[1], "read"))
	{
//		StaticADControl_MCP4651Read(atoi(argv[2]));
	}
	else if (IsEqual(argv[1], "write"))
	{
//		StaticADControl_MCP4651Write(atoi(argv[2]), atoi(argv[3]));
	}
	else if (0 == argv[1] || IsEqual(argv[1], "help") ||
			IsEqual(argv[1], "?"))
   {
	   Printf("====== MCP4651 commands ======\n");
	   Printf(" mcp write [index][value] : meter index 2~3\n");
	   Printf(" mcp read  [index]  :  \n");
//       Printf(" mcp test [index][addr][value][time][period]:  \n");
//       Printf(" mcp random [index][addr]: return the random read value \n");
//       Printf(" mcp continuous [index][addr]: return 32 bit value \n");
   }
	else
	{
		Printf("Invalid param: %s\n", argv[1]);
	}
	return (0);
}

// 漏液检测
int Cmd_ChcekLeaking(int argc, char *argv[])
{
    if (IsEqual(argv[1], "start"))
    {
        if(argv[2])
        {
        	CheckLeaking_SetCheckLeakingReportPeriod(atof(argv[2]));
        	Printf("\nCheck leaking start");
        }
        else
        {
        	 Printf("Invalid param: %s\n", argv[2]);
        }
    }
    else if (IsEqual(argv[1], "stop"))
    {
    	CheckLeaking_SetCheckLeakingReportPeriod(0);
        Printf("\nCheck leaking stop");
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== Sub commands ======\n");
        Printf(" check start [period] : start checking task, period unit (s)\n");
        Printf(" check stop       : stop checking task\n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }

    return (0);
}

//继电器控制
static int Cmd_Relay(int argc, char *argv[])
{
	if (IsEqual(argv[1], "on"))
	{
		if(argv[2])
		{
			RelayMap_On(atoi(argv[2]));
		}
		else
		{
			Printf("\n Invalid Relay Index");
		}
	}
	else if (IsEqual(argv[1], "off"))
	{
		if(argv[2])
		{
			RelayMap_Off(atoi(argv[2]));
		}
		else
		{
			Printf("\n Invalid Relay Index");
		}
	}
	else if (0 == argv[1] || IsEqual(argv[1], "help") ||
			 IsEqual(argv[1], "?"))
	{
		Printf("====== Relay ======\n");
		Printf(" relay on [index] : turn on relay;index:[0/1/2]\n");
		Printf(" relay off [index] : turn off relay;index:[0/1/2]\n");
	}
	else
	{
		Printf("Invalid param: %s\n", argv[1]);
	}
	return (0);
}

//传感器检测
static int Cmd_Sensor(int argc, char *argv[])
{
	if(IsEqual(argv[1], "ad"))
	{
		Printf("ad[0][%d]", TempADCollect_GetAD(0));
		Printf("ad[1][%d]", TempADCollect_GetAD(1));
		Printf("ad[2][%d]", TempADCollect_GetAD(2));
	}
	else if (IsEqual(argv[1], "map"))
	{
//		if(atoi(argv[1]) < 2)
//		{
//			DetectSensorManager_GetSensor(atoi(argv[1]));
//		}
//		else
		{
			DetectSensorManager_GetSensorsMap();
		}
		if(TempADCollect_GetAD(2)>3800)
		{
			Printf("\nSY11 Water [No]\n");
		}
		else
		{
			Printf("\nSY11 Water [Yes]\n");
		}
	}
	else if (0 == argv[1] || IsEqual(argv[1], "help") ||
			 IsEqual(argv[1], "?"))
	{
		Printf("====== Sensor ======\n");
		Printf(" sensor map : \n");
		Printf(" sensor ad : get optical ad value\n");
	}
	else
	{
		Printf("Invalid param: %s\n", argv[1]);
	}
	return (0);
}
/** @} */



int Cmd_Debug(int argc, char *argv[])
{
    if (IsEqual(argv[1], "start"))
    {
    	if(argv[2]&&argv[3]&&argv[4])
    	{
    		Uint8 index = 0;
    		if(IsEqual(argv[2], "x"))
    		{
    			index  = 0;
    		}
    		else if(IsEqual(argv[2], "y"))
    		{
    			index  = 1;
    		}
    		else if(IsEqual(argv[2], "z"))
			{
				index  = 2;
			}
    		Communication_MotorStart(index, atoi(argv[3]), atoi(argv[4]));
    	}
    	else
    	{
    		Printf("Motor Start Err\n");
    	}
    }
    else if (IsEqual(argv[1], "startxyz"))
	{
		if(argv[2]&&argv[3]&&argv[4]&&argv[5])
		{
			Motor3DPos pos;
			pos.xpos = atoi(argv[3]);
			pos.ypos = atoi(argv[4]);
			pos.zpos = atoi(argv[5]);
			Communication_MotorStartXYZ(atoi(argv[2]), pos);
			Printf("Motor StartXYZ\n");
		}
		else
		{
			Printf("Motor StartXYZ Err\n");
		}
	}
    else  if (IsEqual(argv[1], "enable"))
	{
    	if (IsEqual(argv[2], "x"))
    	{
    		Communication_ServoOn(0);
    		Printf("Servo On x\n");
    	}
    	else if (IsEqual(argv[2], "y"))
    	{
    		Communication_ServoOn(1);
    		Printf("Servo On y\n");
    	}
    	else if (IsEqual(argv[2], "z"))
		{
			Communication_ServoOn(2);
			Printf("Servo On x\n");
		}
	}
    else  if (IsEqual(argv[1], "disable"))
	{
		if (IsEqual(argv[2], "x"))
		{
			Communication_ServoOff(0);
			Printf("Servo Off x\n");
		}
		else if (IsEqual(argv[2], "y"))
		{
			Communication_ServoOff(1);
			Printf("Servo Off y\n");
		}
		else if (IsEqual(argv[2], "z"))
		{
			Communication_ServoOff(2);
			Printf("Servo Off x\n");
		}
	}
    else  if (IsEqual(argv[1], "reset"))
	{
    	if (IsEqual(argv[2], "x"))
		{
    		Communication_ResetAxis(0);
			Printf("Axis x Reset\n");
		}
    	else if (IsEqual(argv[2], "y"))
		{
    		Communication_ResetAxis(1);
			Printf("Axis y Reset\n");
		}
		else if (IsEqual(argv[2], "z"))
		{
			Communication_ResetAxis(2);
			Printf("Axis z Reset\n");
		}
	}
    else  if (IsEqual(argv[1], "resetall"))
	{
    	Communication_MotorResetXYZ();
	}
    else  if (IsEqual(argv[1], "stop"))
	{
		if(IsEqual(argv[2], "x"))
		{
			Communication_EmgsStop(0);
			Printf("Axis x Stop\n");
		}
		else if (IsEqual(argv[2], "y"))
		{
			Communication_EmgsStop(1);
			Printf("Axis y Stop\n");
		}
		else if (IsEqual(argv[2], "z"))
		{
			Communication_EmgsStop(2);
			Printf("Axis z Stop\n");
		}
		else
		{
			Communication_EmgsStop(0);
			Communication_EmgsStop(1);
			Communication_EmgsStop(2);
			Printf("Axis All Stop\n");
		}

	}
    else  if (IsEqual(argv[1], "pr"))
	{
		if(IsEqual(argv[2], "x"))
		{
			Communication_SetPRMode(0);
			Printf("Axis x PR Mode\n");
		}
		else if(IsEqual(argv[2], "y"))
		{
			Communication_SetPRMode(1);
			Printf("Axis y PR Mode\n");
		}
		else if(IsEqual(argv[2], "z"))
		{
			Communication_SetPRMode(2);
			Printf("Axis z PR Mode\n");
		}
		else
		{
			Printf("PR Error\n");
		}
	}
    else  if (IsEqual(argv[1], "posget"))
   	{
   		if(IsEqual(argv[2], "x"))
   		{
   			Int32 pos = Communication_GetAxisPosition(0);
   			Printf("Axis x Pos %d\n", pos);
   		}
   		else if(IsEqual(argv[2], "y"))
		{
			Int32 pos = Communication_GetAxisPosition(1);
			Printf("Axis y Pos %d\n", pos);
		}
   		else if(IsEqual(argv[2], "z"))
		{
			Int32 pos = Communication_GetAxisPosition(2);
			Printf("Axis z Pos %d\n", pos);
		}
   		else if(IsEqual(argv[2], "all"))
		{
   			Motor3DPos pos = Communication_MotorGetXYZ();
			Printf("Axis XYZ Pos [%d][%d][%d]\n", pos.xpos, pos.ypos, pos.zpos);
		}
   		else
   		{
   			Printf("Get Pos Error\n");
   		}
   	}
    else  if (IsEqual(argv[1], "posclr"))
	{
		if(IsEqual(argv[2], "x"))
		{
			Communication_ClearAxisPosition(0);
			Printf("Axis x Clear Pos\n");
		}
		else if(IsEqual(argv[2], "y"))
		{
			Communication_ClearAxisPosition(1);
			Printf("Axis y Clear Pos\n");
		}
		else if(IsEqual(argv[2], "z"))
		{
			Communication_ClearAxisPosition(2);
			Printf("Axis z Clear Pos\n");
		}
		else
		{
			Printf("Clear Error\n");
		}
	}
    else  if (IsEqual(argv[1], "di"))
	{
		if(IsEqual(argv[2], "x"))
		{
			Uint16 di = Communication_GetDIStatus(0);
			Printf("Axis x DI [0x%x]\n", di);
		}
		else if(IsEqual(argv[2], "y"))
		{
			Uint16 di = Communication_GetDIStatus(1);
			Printf("Axis y DI [0x%x]\n", di);
		}
		else if(IsEqual(argv[2], "z"))
		{
			Uint16 di = Communication_GetDIStatus(2);
			Printf("Axis z DI [0x%x]\n", di);
		}
		else
		{
			Printf("DI Error\n");
		}
	}
    else  if (IsEqual(argv[1], "do"))
	{
		if(IsEqual(argv[2], "x"))
		{
			Uint16 ret = Communication_GetDOStatus(0);
			Communication_DOReport(ret);
			Printf("Axis x DO [0x%x]\n", ret);
		}
		else if(IsEqual(argv[2], "y"))
		{
			Uint16 ret = Communication_GetDOStatus(1);
			Communication_DOReport(ret);
			Printf("Axis y DO [0x%x]\n", ret);
		}
		else if(IsEqual(argv[2], "z"))
		{
			Uint16 ret = Communication_GetDOStatus(2);
			Communication_DOReport(ret);
			Printf("Axis z DO [0x%x]\n", ret);
		}
		else
		{
			Printf("DO Error\n");
		}
	}
    else  if (IsEqual(argv[1], "td"))
	{
		if(IsEqual(argv[2], "x"))
		{
			Communication_TargetDefine(0, atoi(argv[3]), atoi(argv[4]));
			Printf("Axis x Set Posparam\n");
		}
		else if(IsEqual(argv[2], "y"))
		{
			Communication_TargetDefine(1, atoi(argv[3]), atoi(argv[4]));
			Printf("Axis y Set Posparam\n");
		}
		else if(IsEqual(argv[2], "z"))
		{
			Communication_TargetDefine(2, atoi(argv[3]), atoi(argv[4]));
			Printf("Axis z Set Posparam\n");
		}
		else
		{
			Printf("Axis target define error\n");
		}
	}
    else  if (IsEqual(argv[1], "sd"))
	{
		if(IsEqual(argv[2], "x"))
		{
			Communication_SetTargetParam(0, atoi(argv[3]));
			Printf("Axis x Set Posparam\n");
		}
		else if(IsEqual(argv[2], "y"))
		{
			Communication_SetTargetParam(1, atoi(argv[3]));
			Printf("Axis y Set Posparam\n");
		}
		else if(IsEqual(argv[2], "z"))
		{
			Communication_SetTargetParam(2, atoi(argv[3]));
			Printf("Axis z Set Posparam\n");
		}
		else
		{
			Printf("Axis set pos param error\n");
		}
	}
    else  if (IsEqual(argv[1], "mov"))
	{
		if(IsEqual(argv[2], "x"))
		{
			Communication_MoveTarget(0);
			Printf("Axis x Mov \n");
		}
		else if(IsEqual(argv[2], "y"))
		{
			Communication_MoveTarget(1);
			Printf("Axis y Mov \n");
		}
		else if(IsEqual(argv[2], "z"))
		{
			Communication_MoveTarget(2);
			Printf("Axis z Mov \n");
		}
		else
		{
			Printf("Axis Mov Failed\n");
		}
	}
    else  if (IsEqual(argv[1], "hd"))
    {
    	if(IsEqual(argv[2], "x"))
    	{
    		Communication_HomingDefine(0);
    	}
    	else if(IsEqual(argv[2], "y"))
    	{
    		Communication_HomingDefine(1);
    	}
    	else if(IsEqual(argv[2], "z"))
    	{
    		Communication_HomingDefine(2);
    	}
    	else
    	{
    		Printf("Axis Home Define Failed\n");
    	}
    }
    else  if (IsEqual(argv[1], "firstspeed"))
    {
    	Uint8 index = 0;
    	if(IsEqual(argv[2], "x"))
    	{
    		index = 0;
    	}
    	else if(IsEqual(argv[2], "y"))
    	{
    		index = 1;
    	}
    	else
    	{
    		index = 2;
    	}
    	Communication_SetHomingFirstSpeed(index, atoi(argv[3]));
    	Printf("Set %d First Speed\n", index, atoi(argv[3]));
    }
    else  if (IsEqual(argv[1], "secondspeed"))
	{
    	Uint8 index = 0;
		if(IsEqual(argv[2], "x"))
		{
			index = 0;
		}
		else if(IsEqual(argv[2], "y"))
		{
			index = 1;
		}
		else
		{
			index = 2;
		}
    	Communication_SetHomingSecondSpeed(index, atoi(argv[3]));
    	Printf("Set %d Second Speed\n", index, atoi(argv[3]));
	}
    else  if (IsEqual(argv[1], "save"))
	{

    	Uint8 index = 0;
		if(IsEqual(argv[2], "x"))
		{
			index = 0;
		}
		else if(IsEqual(argv[2], "y"))
		{
			index = 1;
		}
		else
		{
			index = 2;
		}
		Communication_SaveToEPPROM(index);
		Printf("Save %d To EPPROM\n", index);
	}
    else  if (IsEqual(argv[1], "spd"))
	{
    	if(argv[2])
    	{
    		Communication_SetRunSpeed(atoi(argv[2]));
    	}
	}
    else  if (IsEqual(argv[1], "test"))
	{
		if(atoi(argv[2])>200)
		{
			s_getInfoTime = atoi(argv[2]);
			 vTaskResume(s_InfoOutputHandle);
			Printf("Servo Modbus Test Start %dms\n", atoi(argv[2]));
		}
		else
		{
			s_getInfoTime = 0;
			vTaskSuspend(s_InfoOutputHandle);
			Printf("Servo Modbus Test Stop\n");
		}
	}
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== debug commands ======\n");
        Printf(" s enable [x/y/z]: servo on\n");
        Printf(" s disable [x/y/z]:servo off\n");
        Printf(" s pr [x/y/z]: home define\n");
        Printf(" s start [x/y/z][mode][target]: [mode:0-abs,1-rel,2-inc][target] \n");
        System_Delay(10);
        Printf(" s startxyz [mode][xpos][ypos][zpos]: [mode:0-abs,1-rel,2-inc][pos] \n");
        Printf(" s stop [x/y/z]: emgs stop\n");
        Printf(" s reset [x/y/z]: home reset x\n");
        Printf(" s resetall: reset all motor\n");
        Printf(" s hd [x/y/z]: home define\n");
        System_Delay(10);
        Printf(" s td [x/y/z][mode][spd]: set path 1 define,[mode:0-abs,1-rel,2-inc][spd:0-10]\n");
        Printf(" s sd [x/y/z][pos]: set path 1 data\n");
        Printf(" s mov [x/y/z]: move to path 1\n");
        Printf(" s posget [x/y/z]: get current position\n");
        Printf(" s posclr [x/y/z]: clear position to zero\n");
        System_Delay(10);
        Printf(" s di [x/y/z]: DI status, 0x40 -> DI5 Trigger \n");
        Printf(" s do [x/y/z]: DO status with report \n");
        Printf(" s test [period]: Modbus Test perido > 200 \n");
        System_Delay(10);
        Printf(" s firstspeed [x/y/z][speed]: [1-20000] \n");
        Printf(" s secondspeed [x/y/z][speed]: [1-5000] \n");
        Printf(" s save [x/y/z]: save to epprom\n");
        Printf(" s spd [value]: set speed 0-15\n");
    }
    return 0;
}

static int Cmd_SY11(int argc, char *argv[])
{
	if (IsEqual(argv[1], "reset"))
	{
		SyringeManager_Reset(FALSE);
		Printf("SY11 Reset\n");
	}
	else if (IsEqual(argv[1], "stop"))
	{
		SyringeManager_Stop();
		Printf("SY11 Stop\n");
	}
	else if (IsEqual(argv[1], "on"))
	{
		SyringeManager_Debug(TRUE);
		Printf("SY11 On\n");
	}
	else if (IsEqual(argv[1], "off"))
	{
		SyringeManager_Debug(FALSE);
		Printf("SY11 Off\n");
	}
	else if (IsEqual(argv[1], "valve"))
	{
		SyringeManager_OpenValve(atoi(argv[2]));
		Printf("SY11 Open Valve %d\n", atoi(argv[2]));
	}
	else if(IsEqual(argv[1], "e"))
	{
		SyringeManager_Start(atoi(argv[2]), atof(argv[3]), SY11_PUMP);
		Printf("SY11 Start Suck\n");
	}
	else if(IsEqual(argv[1], "d"))
	{
		SyringeManager_Start(atoi(argv[2]), atof(argv[3]), SY11_DRAIN);
		Printf("SY11 Start Drain\n");
	}
	else if(IsEqual(argv[1], "abs"))
	{
		SyringeManager_Start(atoi(argv[2]), atof(argv[3]), SY11_ABS);
		Printf("SY11 Start Abs\n");
	}
	else if (0 == argv[1] || IsEqual(argv[1], "help") ||
			 IsEqual(argv[1], "?"))
	{
		Printf("====== SY11 ======\n");
		Printf(" sy on : start SY11 connect\n");
		Printf(" sy off : stop SY11 connect \n");
		Printf(" sy valve [num]:[num:1-4] \n");
		Printf(" sy reset : \n");
		Printf(" sy stop : \n");
		Printf(" sy [dir][valve][vol] :[e/d/abs][1-4][0-1000ul] \n");
	}
	else
	{
		Printf("Invalid param: %s\n", argv[1]);
	}
	return (0);
}


int Cmd_Temp(int argc, char *argv[])
{
    if (IsEqual(argv[1], "env"))
    {
        Printf("\n EnvironmentTemp :  %f", EnvironmentTemp_Get());
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== temp commands ======\n");
        Printf(" temp env                              :\n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

int Cmd_5175(int argc, char *argv[])
{
    if (IsEqual(argv[1], "set"))
    {
    	AD5175_WriteRDAC(atoi(argv[2]));
    }
    else if (IsEqual(argv[1], "get"))
	{
    	Printf("\n ad5175 current value :  %d", AD5175_ReadRDAC());
	}
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== 5175 commands ======\n");
        Printf(" ad set [value]   :\n");
        Printf(" ad get   :\n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}
