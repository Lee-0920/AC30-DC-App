/*
 * AD5175Driver.c
 *
 *  Created on: 2018年11月19日
 *      Author: Administrator
 */
#include "AD5175Driver.h"
#include "stm32f4xx.h"
#include "Tracer/Trace.h"
#include "Common/Types.h"
#include "System.h"
#include <string.h>
#include <stdlib.h>
//****************命令码定义 ****************
#define AD5175_RESET    0x1000    //设备复位(50TP保存值更新RDAC)
#define AD5175_PREPARE_WRITE_RDACAND50TP    0x1C03    //准备写RDAC+50TP命令

#define AD5175_PREPARE_WRITE_RDAC    0x1C02    //准备写RDAC命令
#define AD5175_WRITE_RDAC(value)    (0x0400 + (value&0x03FF))   //写value进RDAC   //接上一条使用

#define AD5175_PREPARE_READ_RDAC    0x0800    //准备读RDAC命令  //后续直接读出RDAC值?

#define AD5175_STORE_RDAC_TO_50TP    0x0C00    //保存RDAC值至50TP命令   //>=350ms

#define AD5175_PREPARE_READ_50TP_LASTADDR   0x1800    //准备读取50TP最新使用的地址
#define AD5175_READ_50TP_LASTADDR    0x0000    //读取50TP最新使用的地址  //接上一条使用  //后续直接读出地址?

#define AD5175_PREPARE_READ_50TP_DESTADDR_VALUE(addr)   (0x1400 + (addr&0x00FF))    //准备读取50TP目标地址内容  //后续直接读出50TP值?

#define AD5175_PREPARE_READ_CTRL_REG    0x2000   //准备读取控制寄存器值
#define AD5175_READ_CTRL_REG    0x0000   //读取控制寄存器值  //接上一条使用   //后续直接读出控制值?

// **************** 引脚定义 ****************
//#define AD5175_SCL_PIN              GPIO_Pin_0
//#define AD5175_SCL_PORT           GPIOC
//#define AD5175_SCL_RCC              RCC_AHB1Periph_GPIOC
//#define AD5175_SCL_HIGH()         GPIO_SetBits(AD5175_SCL_PORT, AD5175_SCL_PIN)
//#define AD5175_SCL_LOW()          GPIO_ResetBits(AD5175_SCL_PORT, AD5175_SCL_PIN)
//
//#define AD5175_SDA_PIN             GPIO_Pin_1
//#define AD5175_SDA_PORT          GPIOC
//#define AD5175_SDA_RCC            RCC_AHB1Periph_GPIOC
//#define AD5175_SDA_HIGH()       GPIO_SetBits(AD5175_SDA_PORT, AD5175_SDA_PIN)
//#define AD5175_SDA_LOW()        GPIO_ResetBits(AD5175_SDA_PORT, AD5175_SDA_PIN)
//#define AD5175_SDA_READ()      GPIO_ReadInputDataBit(AD5175_SDA_PORT, AD5175_SDA_PIN)
//#define AD5175_SDA_IN()            {GPIOC->MODER&=~(3<<(1*2));GPIOC->MODER|=(0<<(1*2));}
//#define AD5175_SDA_OUT()         {GPIOC->MODER&=~(3<<(1*2));GPIOC->MODER|=(1<<(1*2));}

AD5175Driver s_ad5175;

void AD5175_SCLHigh(AD5175Driver* ad5175)
{
    GPIO_SetBits(ad5175->portSCL, ad5175->pinSCL);
}

void AD5175_SCLLow(AD5175Driver* ad5175)
{
    GPIO_ResetBits(ad5175->portSCL, ad5175->pinSCL);
}

void AD5175_SDAHigh(AD5175Driver* ad5175)
{
    GPIO_SetBits(ad5175->portSDA, ad5175->pinSDA);
}

void AD5175_SDALow(AD5175Driver* ad5175)
{
    GPIO_ResetBits(ad5175->portSDA, ad5175->pinSDA);
}

Uint8 AD5175_SDARead(AD5175Driver* ad5175)
{
    return GPIO_ReadInputDataBit(ad5175->portSDA, ad5175->pinSDA);
}

void AD5175_SDAIn(AD5175Driver* ad5175)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = ad5175->pinSDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ad5175->portSDA, &GPIO_InitStructure);
}

void AD5175_SDAOut(AD5175Driver* ad5175)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = ad5175->pinSDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ad5175->portSDA, &GPIO_InitStructure);
}

//void AD5175_Config(AD5175Driver* ad5175)
//{
//    GPIO_InitTypeDef GPIO_InitStructure;
//    RCC_AHB1PeriphClockCmd(ad5175->rccSCL | ad5175->rccSDA, ENABLE);
//
//    GPIO_InitStructure.GPIO_Pin = ad5175->pinSCL;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(ad5175->portSCL, &GPIO_InitStructure);
//    AD5175_SCLHigh(ad5175);
//
//    GPIO_InitStructure.GPIO_Pin = ad5175->pinSDA;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(ad5175->portSDA, &GPIO_InitStructure);
//    AD5175_SDAHigh(ad5175);
//}

void AD5175_Start(AD5175Driver* ad5175)
{
    AD5175_SDAOut(ad5175);

    //SCL、SDA拉高
    AD5175_SDAHigh(ad5175);
    AD5175_SCLHigh(ad5175);
    System_DelayUs(5);

    //SDA拉低
    AD5175_SDALow(ad5175);
    System_DelayUs(5);

    //SCL拉低
    AD5175_SCLLow(ad5175);
}

void AD5175_Stop(AD5175Driver* ad5175)
{
    AD5175_SDAOut(ad5175);

    //SCL、SDA拉低
    AD5175_SCLLow(ad5175);
    AD5175_SDALow(ad5175);
    System_DelayUs(5);

    //SCL、SDA拉高
    AD5175_SCLHigh(ad5175);
    System_DelayUs(5);
    AD5175_SDAHigh(ad5175);
}

Bool AD5175_WaitAck(AD5175Driver* ad5175)
{
    Uint8 errTimes = 0;

    AD5175_SDAIn(ad5175);

    AD5175_SDAHigh(ad5175);
    System_DelayUs(1);
    AD5175_SCLHigh(ad5175);
    System_DelayUs(1);

    while(AD5175_SDARead(ad5175))
    {
        errTimes++;
        if(errTimes>=250)
        {
            AD5175_Stop(ad5175);
            return FALSE;  //接收应答失败
        }
    }

    AD5175_SCLLow(ad5175);
    return TRUE;   //接收应答成功
}

void AD5175_SendAck(AD5175Driver* ad5175)
{
    AD5175_SCLLow(ad5175);

    AD5175_SDAOut(ad5175);

    AD5175_SDALow(ad5175);
    System_DelayUs(1);
    AD5175_SCLHigh(ad5175);
    System_DelayUs(5);
    AD5175_SCLLow(ad5175);
}

void AD5175_SendNoAck(AD5175Driver* ad5175)
{
    AD5175_SCLLow(ad5175);

    AD5175_SDAOut(ad5175);

    AD5175_SDAHigh(ad5175);
    System_DelayUs(1);
    AD5175_SCLHigh(ad5175);
    System_DelayUs(5);
    AD5175_SCLLow(ad5175);
}

void AD5175_SendByte(AD5175Driver* ad5175, Uint8 byte)
{
    Uint8 i;
    AD5175_SDAOut(ad5175);

    AD5175_SCLLow(ad5175);
    for(i = 0; i < 8; i++)
    {
        if (byte & 0x80)
        {
            AD5175_SDAHigh(ad5175);
        }
        else
        {
            AD5175_SDALow(ad5175);
        }
        byte <<= 1;
        System_DelayUs(1);

        AD5175_SCLHigh(ad5175);
        System_DelayUs(5);

        AD5175_SCLLow(ad5175);
        System_DelayUs(4);
    }
}

Uint8 AD5175_ReadByte(AD5175Driver* ad5175)
{
    Uint8 i,receive = 0;
    AD5175_SDAIn(ad5175);

    for(i = 0; i < 8; i++)
    {
        AD5175_SCLLow(ad5175);
        System_DelayUs(2);
        AD5175_SCLHigh(ad5175);

        receive<<=1;
        if(AD5175_SDARead(ad5175))
        {
            receive++;
        }
        System_DelayUs(1);
    }

    return receive;
}

void AD5175_Init(void)
{
	memset(&s_ad5175, 0 , sizeof(s_ad5175));
	s_ad5175.pinSCL = GPIO_Pin_1;
	s_ad5175.portSCL = GPIOC;
	s_ad5175.rccSCL = RCC_AHB1Periph_GPIOC;

	s_ad5175.pinSDA = GPIO_Pin_2;
	s_ad5175.portSDA = GPIOC;
	s_ad5175.rccSDA = RCC_AHB1Periph_GPIOC;

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(s_ad5175.rccSCL | s_ad5175.rccSDA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = s_ad5175.pinSCL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(s_ad5175.portSCL, &GPIO_InitStructure);
    AD5175_SCLHigh(&s_ad5175);

    GPIO_InitStructure.GPIO_Pin = s_ad5175.pinSDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(s_ad5175.portSDA, &GPIO_InitStructure);
    AD5175_SDAHigh(&s_ad5175);

    TRACE_INFO("\n AD5175 Init Over");
}
//AD5175_ADDR
Uint16 AD5175_ReadWord(void)
{
    Uint8 addr_read = (AD5175_ADDR<<1)+1;
    Uint16 result = 0;

    AD5175_Start(&s_ad5175);
    AD5175_SendByte(&s_ad5175, addr_read);
    if(!AD5175_WaitAck(&s_ad5175))
    {
        TRACE_ERROR("\n ad5175 read word send addr fail : nack");
        return result;
    }

    result += AD5175_ReadByte(&s_ad5175);
    AD5175_SendAck(&s_ad5175);  //发送应答ACK
    result<<=8;
    result +=  AD5175_ReadByte(&s_ad5175);
    AD5175_SendNoAck(&s_ad5175);  //发送NACK

    AD5175_Stop(&s_ad5175);

    return result;
}

Bool AD5175_WriteWord(Uint16 word)
{
    Uint8 addr_wirte = AD5175_ADDR<<1;
    Uint8 msb = (Uint8)((word>>8)&0xFF);
    Uint8 lsb = (Uint8)(word&0xFF);

    AD5175_Start(&s_ad5175);
    AD5175_SendByte(&s_ad5175, addr_wirte);
    if(!AD5175_WaitAck(&s_ad5175))
    {
        TRACE_ERROR("\n ad5175 write word send addr fail : nack");
        return FALSE;
    }

    AD5175_SendByte(&s_ad5175, msb);
    if(!AD5175_WaitAck(&s_ad5175))
    {
        TRACE_ERROR("\n ad5175 write word msb:%X fail : nack", msb);
        return FALSE;
    }

    AD5175_SendByte(&s_ad5175, lsb);
    if(!AD5175_WaitAck(&s_ad5175))
    {
        TRACE_ERROR("\n ad5175 write word lsb:%X fail : nack", lsb);
        return FALSE;
    }

    AD5175_Stop(&s_ad5175);

    return TRUE;
}

Bool AD5175_WriteWords(Uint16* buffer, Uint8 len)
{
    Uint8 i,msb,lsb;
    Uint8 addr_wirte = AD5175_ADDR<<1;

    AD5175_Start(&s_ad5175);
    AD5175_SendByte(&s_ad5175, addr_wirte);
    if(!AD5175_WaitAck(&s_ad5175))
    {
        TRACE_ERROR("\n ad5175 write words send addr fail : nack");
        return FALSE;
    }

    for(i = 0; i < len; i++)
    {
        msb = (Uint8)((buffer[i]>>8)&0xFF);
        lsb = (Uint8)(buffer[i]&0xFF);
        AD5175_SendByte(&s_ad5175, msb);
        if(!AD5175_WaitAck(&s_ad5175))
        {
            TRACE_ERROR("\n ad5175 write words[%d] msb:%X fail : nack", i, msb);
            return FALSE;
        }
        AD5175_SendByte(&s_ad5175, lsb);
        if(!AD5175_WaitAck(&s_ad5175))
        {
            TRACE_ERROR("\n ad5175 write words[%d] lsb:%X fail : nack", i, lsb);
            return FALSE;
        }
    }

    AD5175_Stop(&s_ad5175);

    return TRUE;
}

Uint16 AD5175_ReadRDAC(void)
{
    Uint16 cmd = AD5175_PREPARE_READ_RDAC;
    Uint16 result = 0;

    if(!AD5175_WriteWord(cmd))
    {
        TRACE_ERROR("\n ad5175 read rdac fail.");
        return result;
    }
    result = AD5175_ReadWord();

    TRACE_INFO("\n ad5175 read rdac: %d", result);
    return result;
}

Bool AD5175_WriteRDAC(Uint16 value)
{
    Uint16 cmd[2] = {0};
    cmd[0] = AD5175_PREPARE_WRITE_RDAC;
    cmd[1] = AD5175_WRITE_RDAC(value);

    if(!AD5175_WriteWords(&cmd[0], 2))
    {
        TRACE_ERROR("\n ad5175 write rdac fail");
        return FALSE;
    }
    else
    {
        TRACE_INFO("\n ad5175 write rdac success");
        return TRUE;
    }
}

Bool AD5175_SaveRDACTo50TP(void)
{
    Uint16 cmd = AD5175_STORE_RDAC_TO_50TP;
    AD5175_WriteWord(cmd);

    System_Delay(500);  //需要写入时间

    AD5175_WriteWord(AD5175_PREPARE_READ_CTRL_REG);
    AD5175_WriteWord(AD5175_READ_CTRL_REG);

    if(!(AD5175_ReadWord()))
    {
        TRACE_ERROR("\n ad5175 save 50tp fail");
        return FALSE;
    }
    else
    {
        TRACE_INFO("\n ad5175 save 50tp success");
        return TRUE;
    }
}
