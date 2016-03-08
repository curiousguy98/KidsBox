/********************************************************
*
* @file      [GizWits.cpp]
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

#include "GizWits.h"
#include <MsTimer2.h>

#define M5_VERSION

UART_HandleTypeDef UART_HandleStruct;
Pro_M2W_ReturnInfoTypeDef Pro_M2W_ReturnInfoStruct;
Pro_Wait_AckTypeDef Wait_AckStruct;

uint8_t SN;
uint8_t g_DevStatus[Max_UartBuf];
uint8_t g_P0DataLen;
uint32_t SystemTimeCount;
uint32_t Last_ReportTime = 0;
uint32_t Last_Report_10_Time = 0;
uint8_t timeoutFlag = 0;//Read error package , timeoutFlag 1
uint32_t Reset_TIMER = 0;

#if(GetFrame == 1)
SoftwareSerial mySerial(8, 9); // RX, TX
#endif

//RingBuffer
uint8_t last_data = 0;
RingBuffer u_ring_buff; //环形buff

/*环形缓冲区,全局变量*/
uint8_t packageFlag = 0;
uint16_t dataLen = 0;
uint16_t count = 0;
uint8_t tmp_buf[MAX_P0_LEN] = { 0 }; //MAX_PACKAGE_LEN
uint8_t lastValue = 0;
uint8_t curValue = 0;

uint16_t gaterTime = 0;
uint32_t ReportTimeCount = 0;
extern uint8_t gaterSensorFlag; 

void gokit_timer(void)
{
    SystemTimeCount++;
    gaterTime++;
	
    if(gaterTime >= 1000)
    {
        gaterTime = 0;
        gaterSensorFlag = 1;
    }
	
}

/******************************************************
 *    function    : serialEvent
 *    Description : arduino serial uart receive  interrupt 
 *                  function
 *   
 *    return      : none.
 *    Add by Alex.lin    --2014-12-24
******************************************************/

#ifdef M5_VERSION
void serialEvent1(void)
{
	uint8_t value = 0;
	value = (unsigned char)Serial1.read();
	if(rb_can_write(&u_ring_buff) > 0)
	{
		rb_write(&u_ring_buff, &value, 1);
	}

	mySerial.println(value, HEX);//不加这句容易出BUG
}
#else
void serialEvent(void)
{
	uint8_t value = 0;
	value = (unsigned char)Serial.read();
	if(rb_can_write(&u_ring_buff) > 0)
	{
		rb_write(&u_ring_buff, &value, 1);
	}

	mySerial.println(value, HEX);//不加这句容易出BUG
}
#endif	

uint8_t GizWits_W2D_AckCmdHandle(void)
{
    uint16_t i;  
    Pro_HeadPartTypeDef * Wait_Ack_HeadPart = (Pro_HeadPartTypeDef *)Wait_AckStruct.Cmd_Buff; 
    Pro_HeadPartTypeDef * Recv_HeadPart = (Pro_HeadPartTypeDef *)UART_HandleStruct.Message_Buf; 
    
    //Wait_AckStruct.Flag = 1为检测ACK模式，否则不做处理
    if(Wait_AckStruct.Flag == 1) 
    {
		// #if(DEBUG==1)
			// mySerial.print(F("Wait_Ack_HeadPart->Cmd / SN: "));mySerial.print(Wait_Ack_HeadPart->Cmd,HEX);mySerial.print(F(" / "));mySerial.print(Wait_Ack_HeadPart->SN,HEX);
			// mySerial.println("");
			// mySerial.print(F("Recv_HeadPart->Cmd /SN : "));mySerial.print(Recv_HeadPart->Cmd,HEX);mySerial.print(F(" / "));mySerial.print(Recv_HeadPart->SN,HEX);
			// mySerial.println("");
		// #endif

        //符合对应ACK条件行判断操作 否则是其他cmd,直接跳过
        if((Wait_Ack_HeadPart->Cmd == (Recv_HeadPart->Cmd - 1)) && (Wait_Ack_HeadPart->SN == Recv_HeadPart->SN))
        {
            if(Wait_AckStruct.SendNum < Send_MaxNum)
            {
				// #if(DEBUG==1)
					// mySerial.print(F("Time: "));mySerial.print(SystemTimeCount - Wait_AckStruct.SendTime,DEC);mySerial.print(" ");mySerial.print(SystemTimeCount,DEC);mySerial.print(" ");mySerial.print(Wait_AckStruct.SendTime,DEC);
					// mySerial.println("");
				// #endif
				#if(DEBUG==1)
					mySerial.print(F("[Time: "));mySerial.print(SystemTimeCount - Wait_AckStruct.SendTime,DEC);mySerial.print("]");
				#endif
                if((SystemTimeCount - Wait_AckStruct.SendTime) < Send_MaxTime)
                {
                    memset(&Wait_AckStruct, 0, sizeof(Wait_AckStruct)); //Wait_AckStruct.Flag = 0;
                    return 1; //是收到了对应的ACK包
                }
                else
                {
                    memset(&Wait_AckStruct, 0, sizeof(Wait_AckStruct)); //Wait_AckStruct.Flag = 0;
                    return 3; //是收到了对应的ACK包 但超时
                }
            }
            memset(&Wait_AckStruct, 0, sizeof(Wait_AckStruct)); //Wait_AckStruct.Flag = 0;
            return 0; //放弃接收ACK 允许重新reprot
        }

    }

    return 4;//放不做接收ACK处理 
}


/*******************************************************************************
* Function Name  : UART_SendBuf
* Description    : 向串口发送数据帧
* Input          : buf:数据起始地址； packLen:数据长度； tag=0,不等待ACK；tag=1,等待ACK；
* Output         : None
* Return         : None
* Attention		   : 若等待ACK，按照协议失败重发3次；数据区出现FF，在其后增加55
*******************************************************************************/

void Pro_UART_SendBuf(uint8_t *Buf, uint16_t PackLen, uint8_t Tag)
{
	uint16_t i;

	for(i=0; i < PackLen; i++)
	{
#ifdef M5_VERSION
		Serial1.write(Buf[i]);
		if(i >=2 && Buf[i] == 0xFF) Serial1.write(0x55);    // add 0x55 while across 0xff except head. 		
#else		
		Serial.write(Buf[i]);	
		if(i >=2 && Buf[i] == 0xFF) Serial.write(0x55);    // add 0x55 while across 0xff except head. 
#endif				
	}
	
    //若为主动上报需判断返回的ACK
	if(Tag == 1) 
	{
		Wait_AckStruct.SendTime = SystemTimeCount;
		Wait_AckStruct.SendNum = 0;
		Wait_AckStruct.Flag = 1;
		memcpy(Wait_AckStruct.Cmd_Buff, Buf, PackLen);
	}
	
}

/*******************************************************************************
* Function Name  : UART_SendBuf
* Description    : 向串口发送数据帧
* Input          : buf:数据起始地址； packLen:数据长度； tag=0,不等待ACK；tag=1,等待ACK；
* Output         : None
* Return         : None
* Attention		   : 若等待ACK，按照协议失败重发3次；数据区出现FF，在其后增加55
*******************************************************************************/

void Log_UART_SendBuf(uint8_t *Buf, uint16_t PackLen)
{
	uint16_t i;

	#if(DEBUG==1)
		mySerial.println("MCU   :");
		for(i = 0; i < PackLen; i++)
		{
		  mySerial.print(" "); mySerial.print(Buf[i],HEX);
		}
		mySerial.println("");
	#endif
}

/*******************************************************************************
* Function Name  : exchangeBytes
* Description    : 模拟的htons 或者 ntohs，如果系统支字节序更改可直接替换成系统函数
* Input          : value
* Output         : None
* Return         : 更改过字节序的short数值
* Attention		   : None
*******************************************************************************/
short	exchangeBytes(short	value)
{
	short			tmp_value;
	uint8_t		*index_1, *index_2;
	
	index_1 = (uint8_t *)&tmp_value;
	index_2 = (uint8_t *)&value;
	
	*index_1 = *(index_2+1);
	*(index_1+1) = *index_2;
	
	return tmp_value;
}

/*******************************************************************************
* Function Name  : CheckSum
* Description    : 校验和算法
* Input          : buf:数据起始地址； packLen:数据长度；
* Output         : None
* Return         : 校验码
* Attention		   : None
*******************************************************************************/
uint8_t CheckSum( uint8_t *buf, int packLen )
{
	
  int				i;
	uint8_t		sum;
	
	if(buf == NULL || packLen <= 0) return 0;
	sum = 0;
	for(i=2; i<packLen-1; i++) 
		sum += buf[i];
	
	return sum;

}

void GizWits_init(uint8_t P0_Len)
{
	Pro_HeadPartP0CmdTypeDef *Pro_D2W_ReportStatusStruct = (Pro_HeadPartP0CmdTypeDef *)g_DevStatus;

#ifdef M5_VERSION
	Serial1.begin(9600); 
#else	
	Serial.begin(9600); 
#endif	
	#if(GetFrame==1)
	//自定义引脚通信SoftwareSerial初始
	mySerial.begin(9600);
	#endif
    
    //定时中断初始
    MsTimer2::set(1, gokit_timer); // 1ms period
    MsTimer2::start();

    if(P0_Len > MAX_P0_LEN)
    {
        mySerial.println("Warning P0_Len out of range");
        while(1);
    }    
    //RingBuffer
    rb_new(&u_ring_buff);
    
	memset((uint8_t *)&g_DevStatus, 0, 128);
	memset(&Pro_M2W_ReturnInfoStruct, 0, sizeof(Pro_M2W_ReturnInfoStruct));
	
	Pro_M2W_ReturnInfoStruct.Pro_HeadPart.Head[0] = 0xFF;
	Pro_M2W_ReturnInfoStruct.Pro_HeadPart.Head[1] = 0xFF;
	Pro_M2W_ReturnInfoStruct.Pro_HeadPart.Len = exchangeBytes(sizeof(Pro_M2W_ReturnInfoStruct) - 4);
	Pro_M2W_ReturnInfoStruct.Pro_HeadPart.Cmd = Pro_D2W__GetDeviceInfo_Ack_Cmd;
	memcpy(Pro_M2W_ReturnInfoStruct.Pro_ver, PRO_VER, strlen(PRO_VER));
	memcpy(Pro_M2W_ReturnInfoStruct.P0_ver, P0_VER, strlen(P0_VER));
	memcpy(Pro_M2W_ReturnInfoStruct.Hard_ver, HARD_VER, strlen(HARD_VER));
	memcpy(Pro_M2W_ReturnInfoStruct.Soft_ver, SOFT_VER, strlen(SOFT_VER));
	memcpy(Pro_M2W_ReturnInfoStruct.Product_Key, PRODUCT_KEY, strlen(PRODUCT_KEY));
	Pro_M2W_ReturnInfoStruct.Binable_Time = exchangeBytes(0);
	
	
	g_P0DataLen = P0_Len;
	Pro_D2W_ReportStatusStruct->Pro_HeadPart.Head[0] = 0xFF;
	Pro_D2W_ReportStatusStruct->Pro_HeadPart.Head[1] = 0xFF;
	Pro_D2W_ReportStatusStruct->Pro_HeadPart.Len = exchangeBytes(sizeof(Pro_HeadPartP0CmdTypeDef) - 4);
	Pro_D2W_ReportStatusStruct->Pro_HeadPart.Cmd = 0x0;
	Pro_D2W_ReportStatusStruct->Pro_HeadPart.SN = 0;
	Pro_D2W_ReportStatusStruct->Pro_HeadPart.Flags[0] = 0x0;
	Pro_D2W_ReportStatusStruct->Pro_HeadPart.Flags[1] = 0x0;
	//Pro_D2W_ReportStatusStruct->Action = 0x0;
}

uint8_t Pro_GetFrame()
{
    if(rb_can_read(&u_ring_buff) >= 1)
    {
        if(packageFlag ==0)
        {
            rb_read(&u_ring_buff, &curValue, 1);
           
            if((lastValue == 0xFF)&&(curValue == 0xFF))
            {
                tmp_buf[0] = 0xFF;
                tmp_buf[1] = 0xFF;
                count = 2;
                return 1;
            }
            if((lastValue == 0xFF)&&(curValue == 0x55))
            {
                lastValue = curValue;
                return 1;
            }
            tmp_buf[count] = curValue;
            count ++ ;
            lastValue = curValue;

            if(count ==4)
            {
                dataLen = tmp_buf[2]*256+  tmp_buf[3];
            }
            if(count ==  (dataLen + 4))
            {
                memcpy(UART_HandleStruct.Message_Buf, tmp_buf, dataLen + 4);
                UART_HandleStruct.Message_Len = dataLen + 4;
                
#ifdef PROTOCOL_DEBUG
                mySerial.print(F("[")); mySerial.print(SystemTimeCount, DEC); mySerial.print(F("]")); mySerial.print(F(" GAgentToMCU:")); 
                for(uint8_t i = 0; i < dataLen + 4; i++) 
                {
                    mySerial.print(" "); mySerial.print(tmp_buf[i], HEX); 
                }
                mySerial.println(""); 
#endif
				memset(tmp_buf, 0, (dataLen + 4));
                packageFlag = 1;
                lastValue = curValue =0;
                return 0;
            }
        }
    }
	return 1;

}

uint8_t GizWits_D2W_Resend_AckCmdHandle(void)
{
	//超时及放弃接收ACK
	if(Wait_AckStruct.Flag == 1) 
	{
        if(Wait_AckStruct.SendNum < Send_MaxNum)
        {
            if((SystemTimeCount - Wait_AckStruct.SendTime) > Send_MaxTime)
            {
                //需重发
                for(uint8_t i = 0; i < (sizeof(Pro_HeadPartP0CmdTypeDef) + g_P0DataLen + 1); i++)
                {
#ifdef M5_VERSION
					Serial1.write(Wait_AckStruct.Cmd_Buff[i]);
					if(i >=2 && Wait_AckStruct.Cmd_Buff[i] == 0xFF) Serial1.write(0x55);    // add 0x55 while across 0xff except head. 
#else                	
					Serial.write(Wait_AckStruct.Cmd_Buff[i]);				
					if(i >=2 && Wait_AckStruct.Cmd_Buff[i] == 0xFF) Serial.write(0x55);    // add 0x55 while across 0xff except head. 
#endif							
                }
                Wait_AckStruct.SendTime = SystemTimeCount;
                Wait_AckStruct.SendNum++;
                return 2; //重发包 等待接收ACK
            }
        }
        else
        {
            memset(&Wait_AckStruct, 0, sizeof(Wait_AckStruct)); //Wait_AckStruct.Flag = 0;
            return 1; //结束重发Ack机制
        }
	}
	return 0;
}

/*******************************************************************************
* Function Name  : MessageHandle
* Description    : 
* Input          : None
* Output         : None
* Return         : None
* Attention		   : None
*******************************************************************************/
uint8_t GizWits_MessageHandle(uint8_t * Message_Buf, uint8_t Length_buf)
{
    Pro_HeadPartTypeDef * Recv_HeadPart = NULL;
    uint8_t ret = 0;

    //抓取一包
    Pro_GetFrame();
  
	//ACK超时重发机制
	ret = GizWits_D2W_Resend_AckCmdHandle();
	if(ret == 1)
	{
		#if(DEBUG==1)
			mySerial.println(F("Give up Resend!")); 
		#endif
	}
	else if(ret == 2)
	{
		#if(DEBUG==1)
			mySerial.print(F("[Resend ACK --> Time / Num: "));mySerial.print(Wait_AckStruct.SendTime,DEC);mySerial.print(" / ");mySerial.print(Wait_AckStruct.SendNum,DEC);mySerial.print("]");
			mySerial.println(""); 
		#endif
	}
    
    if(packageFlag)
    {
        //验证校验码
        if(CheckSum(UART_HandleStruct.Message_Buf, UART_HandleStruct.Message_Len) !=
            UART_HandleStruct.Message_Buf[UART_HandleStruct.Message_Len - 1])
		{
            Pro_W2D_ErrorCmdHandle(Error_AckSum, 0); 
			packageFlag = 0;
			return 2;		
		}    

		//检测返回ACK状态
		ret = GizWits_W2D_AckCmdHandle();
		if(ret == 1)
		{
			#if(DEBUG==1)
				mySerial.println(F("ACK: SUCCESS! ..."));
			#endif
		}
		else if(ret == 2)
		{
			#if(DEBUG==1)
				mySerial.println(F("ACK: Report Again! ..."));
			#endif
		}
		 else if(ret == 3)
		 {
			#if(DEBUG==1)
				mySerial.println(F("ACK: SUCCESS--but--Time out! ..."));
			#endif
		 }

		Recv_HeadPart = (Pro_HeadPartTypeDef *)UART_HandleStruct.Message_Buf;
		switch (Recv_HeadPart->Cmd)
		{
			case Pro_W2D_GetDeviceInfo_Cmd:
				Pro_W2D_GetMcuInfo();
				break;
			case Pro_W2D_P0_Cmd:
				{
					switch(UART_HandleStruct.Message_Buf[sizeof(Pro_HeadPartTypeDef)])
					{
						case P0_W2D_Control_Devce_Action:
							{
								Pro_W2D_CommonCmdHandle();
								memcpy(Message_Buf, UART_HandleStruct.Message_Buf+sizeof(Pro_HeadPartP0CmdTypeDef), Length_buf); 
                                memset(&UART_HandleStruct.Message_Buf, 0, UART_HandleStruct.Message_Len); 
                                packageFlag = 0; 
								return 0;						 
							}
						case P0_W2D_ReadDevStatus_Action:
							Pro_W2D_ReadDevStatusHandle();
							break;
						default:
							break;
					}
				}
				break;
			case Pro_W2D_P0_Ack_Cmd:
				break; 
			case Pro_W2D_Heartbeat_Cmd:							
				Pro_W2D_CommonCmdHandle();
				break;		
			case Pro_W2D_ControlWifi_Config_Ack_Cmd:
				break;
			case Pro_W2D_ResetWifi_Ack_Cmd:
				break; 			
			case Pro_W2D_ReportWifiStatus_Cmd:
				Pro_W2D_WifiStatusHandle();
				break;
			case Pro_W2D_ReportMCUReset_Cmd:
				Pr0_W2D_RequestResetDeviceHandle();
				break;
			case Pro_W2D_ErrorPackage_Cmd:
                Pro_W2D_ErrorCmdHandle(Error_Other, 1); 
				break;
			default:
                Pro_W2D_ErrorCmdHandle(Error_Cmd, 0); 
				break;
		}	
        memset(&UART_HandleStruct.Message_Buf, 0, UART_HandleStruct.Message_Len); 
        packageFlag = 0;
	}
    
    return 1;
}

/*******************************************************************************
* Function Name  : Pro_GetMcuInfo
* Description    : WiFi模组请求设备信息
* Input          : None
* Output         : None
* Return         : None
* Attention		   : None
*******************************************************************************/
void Pro_W2D_GetMcuInfo(void)
{
	Pro_CommonCmdTypeDef Pro_CommonCmdStruct;
	uint8_t i = 0;
	memcpy(&Pro_CommonCmdStruct, UART_HandleStruct.Message_Buf, sizeof(Pro_CommonCmdStruct));

	Pro_M2W_ReturnInfoStruct.Pro_HeadPart.SN = Pro_CommonCmdStruct.Pro_HeadPart.SN;
	Pro_M2W_ReturnInfoStruct.Sum = CheckSum((uint8_t *)&Pro_M2W_ReturnInfoStruct, sizeof(Pro_M2W_ReturnInfoStruct));
	Pro_UART_SendBuf((uint8_t *)&Pro_M2W_ReturnInfoStruct,sizeof(Pro_M2W_ReturnInfoStruct), 0);

#if(DEBUG==1)
	mySerial.print("[");mySerial.print(SystemTimeCount,DEC);mySerial.print("]");mySerial.print(" MCU   :");
	for(i = 0; i < sizeof(Pro_M2W_ReturnInfoStruct); i++)
	{
	  mySerial.print(" "); mySerial.print(*((uint8_t *)&Pro_M2W_ReturnInfoStruct + i),HEX);
	}
	mySerial.println("");
#endif

// 	Log_UART_SendBuf((uint8_t *)&Pro_M2W_ReturnInfoStruct,sizeof(Pro_M2W_ReturnInfoStruct));
// 	
//  /******************************????*********************************************/	
// 	printf("W2D_GetMcuInfo...\r\n");
// 	printf("PRO_VER:"); 		printf(PRO_VER); 			printf("\r\n");
// 	printf("P0_VER:");			printf(P0_VER);				printf("\r\n");
// 	printf("P0_VER:");      printf(HARD_VER);			printf("\r\n");
// 	printf("SOFT_VER:");    printf(SOFT_VER);			printf("\r\n");
// 	printf("PRODUCT_KEY:"); printf(PRODUCT_KEY);  printf("\r\n");
// /***********************************************************************************/	   
}


void Pro_W2D_ReadDevStatusHandle(void)
{
    Pro_D2W_ReportDevStatusHandle();
}

/*******************************************************************************

* Function Name  : Pro_Pro_W2D_Heartbeat
* Description    : 
* 1，WiFi模组与设备MCU的心跳(4.2)
* 2，设备MCU通知WiFi模组进入配置模式(4.3)
* 3，设备MCU重置WiFi模组(4.4)
* 4, WiFi模组请求重启MCU(4.6)
* 5, WiFi模组请求重启MCU ( 4.9 WiFi模组主动上报当前的状态)
* 6，设备MCU回复 (WiFi模组控制设备)
* 4.6 	WiFi模组请求重启MCU
* 4.9 	Wifi模组回复
* 4.10  设备MCU回复										
* Input          : None
* Output         : None
* Return         : None
* Attention		   : None
*******************************************************************************/
void Pro_W2D_CommonCmdHandle(void)
{
	Pro_CommonCmdTypeDef Pro_CommonCmdStruct;
	uint8_t i = 0;
	
	memcpy(&Pro_CommonCmdStruct, UART_HandleStruct.Message_Buf, sizeof(Pro_CommonCmdStruct));	
    Pro_CommonCmdStruct.Pro_HeadPart.Len = exchangeBytes(sizeof(Pro_CommonCmdTypeDef) - 4); 
	Pro_CommonCmdStruct.Pro_HeadPart.Cmd = Pro_CommonCmdStruct.Pro_HeadPart.Cmd + 1;
	Pro_CommonCmdStruct.Sum = CheckSum((uint8_t *)&Pro_CommonCmdStruct, sizeof(Pro_CommonCmdStruct));	
	Pro_UART_SendBuf((uint8_t *)&Pro_CommonCmdStruct, sizeof(Pro_CommonCmdStruct), 0);	

#if(DEBUG==1)
		mySerial.print("[");mySerial.print(SystemTimeCount,DEC);mySerial.print("]");mySerial.print(" MCU   :");
		for(i = 0; i < (sizeof(Pro_CommonCmdStruct)); i++)
		{
		  mySerial.print(" "); mySerial.print(*((uint8_t *)&Pro_CommonCmdStruct + i),HEX);
		}
		mySerial.println("");
#endif
}

/*******************************************************************************
* Function Name  : Pro_W2D_WifiStatusHandle
* Description    : 将WiFi的状态保存到 Pro_W2D_WifiStatusStruct中。并回复ACK
* Input          : None
* Output         : None
* Return         : None
* Attention		   : None
*******************************************************************************/
void (*callBackFunc)(uint16_t);			
void Pro_W2D_WifiStatusHandle(void)
{
	Pro_W2D_WifiStatusTypeDef Pro_W2D_WifiStatusStruct;
	
	memcpy(&Pro_W2D_WifiStatusStruct, UART_HandleStruct.Message_Buf, sizeof(Pro_W2D_WifiStatusStruct));	
	Pro_W2D_CommonCmdHandle();
    callBackFunc = GizWits_WiFiStatueHandle;
    (*callBackFunc)(exchangeBytes(Pro_W2D_WifiStatusStruct.Wifi_Status)); 
    
} 

/*******************************************************************************
* Function Name  : Pr0_W2D_RequestResetDeviceHandle
* Description    : WiFi模组请求复位设备MCU，MCU回复ACK，并执行设备复位
* Input          : None
* Output         : None
* Return         : None
* Attention		   : None
*******************************************************************************/	
void( * resetFunc) (void) = 0;
void Pr0_W2D_RequestResetDeviceHandle(void)
{	
	Pro_W2D_CommonCmdHandle();

	#if(DEBUG==1)
		mySerial.println(F("W2D_RequestResetDevice..."));
	#endif
	
	//为了避免WiFi模组没有收到确认而重发指令而造成MCU多次重启， 故MCU回复WiFi模组后需等待600毫秒再进行重启
	Reset_TIMER = SystemTimeCount;
	if((SystemTimeCount - Reset_TIMER) > RESTDEV_TIMER)
	{
//		 resetFunc();//会使系统死机需验证
	}
/****************************MCU RESTART****************************/	
	
}

/*******************************************************************************
* Function Name  : Pro_W2D_ErrorCmdHandle
* Description    : WiFi发送收到非法信息通知，设备MCU回复ACK，并执行相应的动作
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/

void Pro_W2D_ErrorCmdHandle(Error_PacketsTypeDef Error_Type, uint8_t flag)
{
	Pro_ErrorCmdTypeDef           	 Pro_ErrorCmdStruct;       //4.7 ??????

    memcpy(&Pro_ErrorCmdStruct, UART_HandleStruct.Message_Buf, sizeof(Pro_ErrorCmdStruct));
    if(flag == 1)
    {
        goto Print_O;
    }
	
    Pro_ErrorCmdStruct.Pro_HeadPart.Head[0] = 0xFF; 
    Pro_ErrorCmdStruct.Pro_HeadPart.Head[1] = 0xFF; 
    Pro_ErrorCmdStruct.Pro_HeadPart.Len = exchangeBytes(sizeof(Pro_ErrorCmdStruct) - 4); 
    Pro_ErrorCmdStruct.Pro_HeadPart.Cmd = Pro_D2W_ErrorPackage_Ack_Cmd; 
    Pro_ErrorCmdStruct.Error_Packets = Error_Type;
    Pro_ErrorCmdStruct.Sum = CheckSum((uint8_t *)&Pro_ErrorCmdStruct, sizeof(Pro_ErrorCmdStruct)); 
    Pro_UART_SendBuf((uint8_t *)&Pro_ErrorCmdStruct, sizeof(Pro_ErrorCmdStruct), 0); 

	#if(DEBUG==1)
    mySerial.print(F("Error :")); mySerial.println(Error_Type, HEX); 
		mySerial.print("[");mySerial.print(SystemTimeCount,DEC);mySerial.print("]");mySerial.print(" MCU   :");
		for(uint8_t i = 0; i < (sizeof(Pro_ErrorCmdStruct)); i++)
		{
		  mySerial.print(" "); mySerial.print(*((uint8_t *)&Pro_ErrorCmdStruct + i),HEX);
		}
		mySerial.println("");
	#endif
    
    return;

Print_O:
	/*************************错误类型*****************************/
	
	switch (Pro_ErrorCmdStruct.Error_Packets)
	{
		case Error_AckSum:

			#if(DEBUG==1)
				mySerial.println(F("ACK : Error_AckSum OK"));
			#endif
			
			break;
		case Error_Cmd:

			#if(DEBUG==1)
            mySerial.println(F("ACK : Error_Cmd OK")); 
			#endif
			
			break;
		case Error_Other:

			#if(DEBUG==1)
            mySerial.println(F("ACK : Error_Other OK")); 
			#endif
			break;
		default:
			#if(DEBUG==1)
            mySerial.println(F("ACK : Error! ")); 
			#endif
			break;
	}	
}	

void Pro_D2W_ReportDevStatusHandle(void)
{
	uint8_t i = 0;
	Pro_HeadPartP0CmdTypeDef *Pro_D2W_ReportStatusStruct = (Pro_HeadPartP0CmdTypeDef *)g_DevStatus;
    memcpy(Pro_D2W_ReportStatusStruct, UART_HandleStruct.Message_Buf, sizeof(Pro_HeadPartTypeDef)); 
        Pro_D2W_ReportStatusStruct->Pro_HeadPart.Cmd = Pro_D2W_P0_Ack_Cmd; 
		Pro_D2W_ReportStatusStruct->Pro_HeadPart.Len = exchangeBytes((sizeof(Pro_HeadPartP0CmdTypeDef)+g_P0DataLen+1) - 4);
        Pro_D2W_ReportStatusStruct->Action = P0_D2W_ReadDevStatus_Action_ACK; 
		g_DevStatus[sizeof(Pro_HeadPartP0CmdTypeDef) + g_P0DataLen] = CheckSum(g_DevStatus, sizeof(Pro_HeadPartP0CmdTypeDef) + g_P0DataLen + 1);
		Pro_UART_SendBuf((uint8_t *)Pro_D2W_ReportStatusStruct, sizeof(Pro_HeadPartP0CmdTypeDef)+g_P0DataLen+1, 0);
		
#if(DEBUG==1)
	mySerial.print("[");mySerial.print(SystemTimeCount,DEC);mySerial.print("]");mySerial.print(" MCU   :");
	for(i = 0; i < (sizeof(Pro_HeadPartP0CmdTypeDef)+g_P0DataLen+1); i++)
	{
	  mySerial.print(" "); mySerial.print(*((uint8_t *)Pro_D2W_ReportStatusStruct + i),HEX);
	}
	mySerial.println("");
#endif

}

void GizWits_D2WResetCmd(void)
{
	uint8_t i = 0;
	Pro_CommonCmdTypeDef Pro_D2WReset;
	
	Pro_D2WReset.Pro_HeadPart.Head[0] = 0xFF;
	Pro_D2WReset.Pro_HeadPart.Head[1] = 0xFF;
	Pro_D2WReset.Pro_HeadPart.Len = exchangeBytes(sizeof(Pro_CommonCmdTypeDef) - 4);
	Pro_D2WReset.Pro_HeadPart.Cmd = Pro_D2W_ResetWifi_Cmd;
	Pro_D2WReset.Pro_HeadPart.SN = SN++;
	Pro_D2WReset.Pro_HeadPart.Flags[0] = 0x00;
	Pro_D2WReset.Pro_HeadPart.Flags[1] = 0x00;
	Pro_D2WReset.Sum = CheckSum((uint8_t *)&Pro_D2WReset, sizeof(Pro_CommonCmdTypeDef));
	Pro_UART_SendBuf((uint8_t *)&Pro_D2WReset, sizeof(Pro_CommonCmdTypeDef), 1); //最后一位为 4.3/4.4/4.9 的重发机制开关
	
#if(DEBUG==1)
	mySerial.print("[");mySerial.print(SystemTimeCount,DEC);mySerial.print("]");mySerial.print(" MCU   :");
	for(i = 0; i < (sizeof(Pro_CommonCmdTypeDef)); i++)
	{
	  mySerial.print(" "); mySerial.print(*((uint8_t *)&Pro_D2WReset + i),HEX);
	}
	mySerial.println("");
#endif
}

void GizWits_D2WConfigCmd(uint8_t WiFi_Mode)
{
	uint8_t i = 0;
	Pro_D2W_ConfigWifiTypeDef Pro_D2WConfigWiFiMode;
	
	Pro_D2WConfigWiFiMode.Pro_HeadPart.Head[0] = 0xFF;
	Pro_D2WConfigWiFiMode.Pro_HeadPart.Head[1] = 0xFF;
	Pro_D2WConfigWiFiMode.Pro_HeadPart.Len = exchangeBytes(sizeof(Pro_D2W_ConfigWifiTypeDef) - 4);
	Pro_D2WConfigWiFiMode.Pro_HeadPart.Cmd = Pro_D2W_ControlWifi_Config_Cmd;
    Pro_D2WConfigWiFiMode.Pro_HeadPart.SN = SN++;
	Pro_D2WConfigWiFiMode.Pro_HeadPart.Flags[0] = 0x00;
	Pro_D2WConfigWiFiMode.Pro_HeadPart.Flags[1] = 0x00;
	Pro_D2WConfigWiFiMode.Config_Method = WiFi_Mode;
	Pro_D2WConfigWiFiMode.Sum = CheckSum((uint8_t *)&Pro_D2WConfigWiFiMode, sizeof(Pro_D2W_ConfigWifiTypeDef));
	Pro_UART_SendBuf((uint8_t *)&Pro_D2WConfigWiFiMode, sizeof(Pro_D2W_ConfigWifiTypeDef), 1); //最后一位为 4.3/4.4/4.9 的重发机制开关
	
#if(DEBUG==1)
	mySerial.print("[");mySerial.print(SystemTimeCount,DEC);mySerial.print("]");mySerial.print(" MCU   :");
	for(i = 0; i < (sizeof(Pro_D2W_ConfigWifiTypeDef)); i++)
	{
	  mySerial.print(" "); mySerial.print(*((uint8_t *)&Pro_D2WConfigWiFiMode + i),HEX);
	}
	mySerial.println("");
#endif
}

void GizWits_DevStatusUpgrade(uint8_t * P0_Buff, uint32_t Time, uint8_t flag, uint8_t ConfigFlag)
{
	uint8_t i = 0;
	uint8_t Report_Flag = 0;
	Pro_HeadPartP0CmdTypeDef *Pro_D2W_ReportStatusStruct = (Pro_HeadPartP0CmdTypeDef *)g_DevStatus;
	
    //等待上一次主动上报后判断ACK的状态，flg = 1：放弃主动上报 flg = 0：可以主动上报
  	if( ConfigFlag == 1 || Wait_AckStruct.Flag == 1)
	{
        return; 
	}
    if(flag == 1) 
    {
        Report_Flag = 1;
        goto Report; 
    }
	
    //设备的状态的变化是由于用户触发或环境变化所产生的， 其发送的频率不能快于2秒每次
    if((2 * 1000) < (SystemTimeCount - Last_ReportTime)) 
    {
        if(memcmp(g_DevStatus + sizeof(Pro_HeadPartP0CmdTypeDef), P0_Buff, g_P0DataLen) != 0)
        {
            Report_Flag = 1;
        }
    }

    //每隔十分钟定时主动上报
    if((Time * 6 * 10) < (SystemTimeCount - Last_Report_10_Time)) 
	{
        mySerial.println("%%%%%%%%%%%%%%%10 minutes regular reporting%%%%%%%%%%%%%%%"); 
		Report_Flag = 1;
        Last_Report_10_Time = SystemTimeCount; 
	}
	
Report:
	if(Report_Flag == 1)
	{
        memcpy(g_DevStatus + sizeof(Pro_HeadPartP0CmdTypeDef), P0_Buff, g_P0DataLen);

        Pro_D2W_ReportStatusStruct->Pro_HeadPart.Len = exchangeBytes(sizeof(Pro_HeadPartP0CmdTypeDef) + 1 + g_P0DataLen - 4);
        Pro_D2W_ReportStatusStruct->Pro_HeadPart.Cmd = Pro_D2W_P0_Cmd;
        Pro_D2W_ReportStatusStruct->Pro_HeadPart.SN = SN++;
        Pro_D2W_ReportStatusStruct->Action = P0_D2W_ReportDevStatus_Action;
        g_DevStatus[sizeof(Pro_HeadPartP0CmdTypeDef) + g_P0DataLen] = CheckSum(g_DevStatus, sizeof(Pro_HeadPartP0CmdTypeDef) + g_P0DataLen + 1);
        Pro_UART_SendBuf((uint8_t *)Pro_D2W_ReportStatusStruct, sizeof(Pro_HeadPartP0CmdTypeDef) + g_P0DataLen + 1, 1);//最后一位为 4.3/4.4/4.9 的重发机制开关

        Last_ReportTime = SystemTimeCount;

#if(DEBUG==1)
		mySerial.print("[");mySerial.print(SystemTimeCount,DEC);mySerial.print("]");mySerial.print(" MCU   :");
		for(i = 0; i < (sizeof(Pro_HeadPartP0CmdTypeDef) + g_P0DataLen + 1); i++)
		{
		  mySerial.print(" "); mySerial.print(g_DevStatus[i],HEX);
		}
		mySerial.println("");
#endif

	}
	return;
}
