/********************************************************
*
* @file      [GizWits.h]
* @author    [True]
* @version   V2.3
* @date      2015-07-06
*
* @brief     机智云 只为智能硬件而生
*            Gizwits Smart Cloud  for Smart Products
*            链接｜增值｜开放｜中立｜安全｜自有｜自由｜生态
*            www.gizwits.com
*
*********************************************************/
#ifndef _GIZWITS_H
#define _GIZWITS_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <ringbuffer.h>

#define PROTOCOL_DEBUG
#define DEBUG				1	//arduino打印开关

#define	PRO_VER				"00000004"
#define	P0_VER				"00000004"
#define	HARD_VER			"02030000"
#define	SOFT_VER			"02030001"
#define	PRODUCT_KEY		"6f3074fe43894547a4f1314bd7e3ae0b"

#define MAX_P0_LEN			128 	 		 //p0˽¾ޗ³¤¶ƍ
#define MAX_PACKAGE_LEN		(MAX_P0_LEN*2) 	 //˽¾ݻº³戸خ´󳤶ƍ
#define MAX_RINGBUFFER_LEN	MAX_PACKAGE_LEN  //»·ю»º³戸خ´󳤶ƍ
#define Max_UartBuf			100

#define USART2_RX_BUF_BOUND	Max_UartBuf-1
#define RESTDEV_TIMER		600
#define SoftAp_Mode			0x01
#define AirLink_Mode		0x02
#define Send_MaxTime   		200    
#define Send_MaxNum    		2
extern SoftwareSerial mySerial;
extern uint32_t SystemTimeCount;

//所有协议指令集合
typedef enum
{
	Pro_W2D_GetDeviceInfo_Cmd 					= 0x01,
	Pro_D2W__GetDeviceInfo_Ack_Cmd				= 0x02,

	Pro_W2D_P0_Cmd 								= 0x03,
	Pro_D2W_P0_Ack_Cmd							= 0x04,

	Pro_D2W_P0_Cmd 								= 0x05,
	Pro_W2D_P0_Ack_Cmd							= 0x06,

	Pro_W2D_Heartbeat_Cmd 						= 0x07,
	Pro_D2W_heartbeatAck_Cmd					= 0x08,

	Pro_D2W_ControlWifi_Config_Cmd 				= 0x09,
	Pro_W2D_ControlWifi_Config_Ack_Cmd			= 0x0A,

	Pro_D2W_ResetWifi_Cmd 						= 0x0B,
	Pro_W2D_ResetWifi_Ack_Cmd					= 0x0C,

	Pro_W2D_ReportWifiStatus_Cmd 				= 0x0D,
	Pro_D2W_ReportWifiStatus_Ack_Cmd			= 0x0E,

	Pro_W2D_ReportMCUReset_Cmd					= 0x0F,
	Pro_W2D_ReportMCUReset_Ack_Cmd				= 0x10,

	Pro_W2D_ErrorPackage_Cmd 					= 0x11,
	Pro_D2W_ErrorPackage_Ack_Cmd				= 0x12,
	
}Pro_CmdTypeDef;

//设备串口通信
typedef struct	
{
	uint8_t            				Message_Buf[Max_UartBuf]; //处理接收到指令的Buf
	uint8_t             			Message_Len;	            //处理信息长度
}UART_HandleTypeDef;

/******************************************************
* ACK 回复参数
* SendTime 
********************************************************/
typedef struct	
{
    uint32_t        SendTime; 
	uint8_t			SendNum;
	uint8_t			Flag;
	uint8_t			Cmd_Buff[Max_UartBuf];
}Pro_Wait_AckTypeDef;


/******************************************************
* 协议的公用部分
********************************************************/
typedef struct	
{
	uint8_t							Head[2];
	uint16_t						Len;
	uint8_t							Cmd;
	uint8_t							SN;
	uint8_t							Flags[2];
	
}Pro_HeadPartTypeDef;

/******************************************************
* 4.1  WiFi模组请求设备信息
********************************************************/
typedef struct	
{
	Pro_HeadPartTypeDef  					Pro_HeadPart;
	uint8_t									Pro_ver[8];
	uint8_t									P0_ver[8];
	uint8_t									Hard_ver[8];
	uint8_t									Soft_ver[8];
	uint8_t									Product_Key[32];
	uint16_t								Binable_Time;
	uint8_t									Sum;
	
}Pro_M2W_ReturnInfoTypeDef;

/*****************************************************
*       通用命令，心跳、ack等可以复用此帧            *
******************************************************
* 4.2 	WiFi模组与设备MCU的心跳
* 4.4 	设备MCU重置WiFi模组 
* 4.6 	WiFi模组请求重启MCU
* 4.9 	Wifi模组回复
* 4.10  设备MCU回复
******************************************************/
typedef struct	
{
	Pro_HeadPartTypeDef    				Pro_HeadPart;
	uint8_t							  	Sum;
}Pro_CommonCmdTypeDef;


/******************************************************
* 4.3 设备MCU通过WiFi模组进入配置模式
********************************************************/
typedef struct	
{
	Pro_HeadPartTypeDef  				Pro_HeadPart;
	uint8_t                 			Config_Method;
	uint8_t							  	Sum;
}Pro_D2W_ConfigWifiTypeDef; 


/*****************************************************
*WiFi工作模式宏定义
******************************************************/
#define			Wifi_SoftAPMode       		(uint8_t)(1<<0)		//是否开启SoftAP 模式，0:关闭，1：开启
#define			Wifi_StationMode      	    (uint8_t)(1<<1)		//是否开启Station模式，0:关闭，1：开启
#define			Wifi_ConfigMode       		(uint8_t)(1<<2)		//是否开启配置模式，0:关闭，1：开启
#define			Wifi_BindingMode     		(uint8_t)(1<<3)		//是否开启绑定模式，0:关闭，1：开启
#define			Wifi_ConnRouter				(uint8_t)(1<<4)		//WiFi模组是否成功连接到路由器，0:关闭，1：开启
#define			Wifi_ConnClouds				(uint8_t)(1<<5)		//WiFi模组是否成功连接到云端，0:关闭，1：开启
	
/*****************************************************
* 4.5 WiFi模组向设备MCU通知WiFi模组工作状态的变化
******************************************************/
typedef struct	
{
	Pro_HeadPartTypeDef    				Pro_HeadPart;
	uint16_t                 			Wifi_Status;
	uint8_t							  	Sum;
}Pro_W2D_WifiStatusTypeDef;

/*****************************************************
* 非法信息通知枚举列表
******************************************************/
typedef enum
{
	Error_AckSum						= 0x01,      			//校验和错误      
	Error_Cmd 	 						= 0x02,				//命令不可识别
	Error_Other							= 0x03,    			//其他错误    

}Error_PacketsTypeDef;


/*****************************************************
* 4.7 非法消息通知
******************************************************/
typedef struct	
{
	Pro_HeadPartTypeDef						Pro_HeadPart;
	uint8_t             					Error_Packets;
	uint8_t									Sum;
	
}Pro_ErrorCmdTypeDef;


/*****************************************************
* P0 command 列表
******************************************************/
typedef enum
{
	P0_W2D_Control_Devce_Action 		= 0x01,
	P0_W2D_ReadDevStatus_Action 		= 0x02,
	P0_D2W_ReadDevStatus_Action_ACK 	= 0x03,
	P0_D2W_ReportDevStatus_Action   	= 0X04,
	
}P0_ActionTypeDef;


/******************************************************
* 带P0指令的公共部分
********************************************************/
typedef struct	
{
	Pro_HeadPartTypeDef  		Pro_HeadPart;
    uint8_t                     Action; 
}Pro_HeadPartP0CmdTypeDef;

void Pro_W2D_GetMcuInfo(void);
void Pro_W2D_CommonCmdHandle(void);
void Pro_W2D_WifiStatusHandle(void);
void Pr0_W2D_RequestResetDeviceHandle(void);
void Pro_W2D_ErrorCmdHandle(Error_PacketsTypeDef Error_Type, uint8_t flag); 
void Pro_W2D_ReadDevStatusHandle(void);
void Pro_D2W_ReportDevStatusHandle(void);
void Pro_UART_SendBuf(uint8_t *Buf, uint16_t PackLen, uint8_t Tag);
void Log_UART_SendBuf(uint8_t *Buf, uint16_t PackLen);
short exchangeBytes(short	value);
uint8_t CheckSum( uint8_t *buf, int packLen );
void GizWits_init(uint8_t P0_Len);
uint8_t Pro_GetFrame(void); 
void GizWits_D2WResetCmd(void);
void GizWits_D2WConfigCmd(uint8_t WiFi_Mode);
void GizWits_DevStatusUpgrade(uint8_t * P0_Buff, uint32_t Time, uint8_t flag, uint8_t ConfigFlag); 
void GizWits_WiFiStatueHandle(uint16_t wifiStatue); 
uint8_t GizWits_D2W_Resend_AckCmdHandle(void);
uint8_t GizWits_W2D_AckCmdHandle(void);
uint8_t GizWits_MessageHandle(uint8_t * Message_Buf, uint8_t Length); 

#endif
