#include "sys.h"
#include "usart.h"	  

#include "device_info.h" 
#include "stmflash.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif


//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

 
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u8 USART_TX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  
  
void uart_init(u32 bound){
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
		USART_DeInit(USART1);  //复位串口1
	
		//USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA9
   
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA10

		//Usart1 NVIC 配置

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
		//USART 初始化设置

		USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

    USART_Init(USART1, &USART_InitStructure);     //初始化串口
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
    USART_Cmd(USART1, ENABLE);                    //使能串口 

}

void USART1_Send_Hex(unsigned char Hex)
{
	USART_SendData(USART1, Hex);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void USART1_Send_PartialCommand(unsigned char *String)
{
	for(;*String!='\0';String++)
	{
		USART1_Send_Hex(*String);
	}
}

////串口1DMA方式发送中断  
//void DMA1_Channel4_IRQHandler(void)  
//{  
//    //清除标志位  
//    DMA_ClearFlag(DMA1_FLAG_TC4);  
//    //DMA_ClearITPendingBit(DMA1_FLAG_TC4);  
//    //DMA1->IFCR |= DMA1_FLAG_TC4;  
//    //关闭DMA  
////    DMA_Cmd(DMA1_Channel4,DISABLE);  
//    //DMA1_Channel4->CCR &= ~(1<<0);  
//  
//    //允许再次发送  
////    Flag_Uart_Send = 0;  
//}  

u16 GetRecSwtich(u8* RXbuff)
{
	u16 temp = 0xAA00;
	temp = temp | ((RXbuff[6]-'0')<<7 );
	temp = temp | ((RXbuff[7]-'0')<<6 );
	temp = temp | ((RXbuff[8]-'0')<<5 );
	temp = temp | ((RXbuff[9]-'0')<<4 );
	temp = temp | ((RXbuff[10]-'0')<<3 );
	temp = temp | ((RXbuff[11]-'0')<<2 );
	temp = temp | ((RXbuff[12]-'0')<<1 );
	temp = temp | ((RXbuff[13]-'0')<<0 );
	
	return temp;
}

u8 len;
void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART1);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		} 

		if(USART_RX_STA&0x8000)	   //判断是否接收完数据
		{					   
			len=USART_RX_STA&0x3f;//得到此次接收到的数据长度   前两位为状态位 所以要与去	

			if(Check_cmd(USART_RX_BUF)==SW)
			{
				//printf("AT+SW=OK\r\n\r\n\r\n");	
//				Sleep(200);
				FLAH_BUFF0[2] = GetRecSwtich(USART_RX_BUF);//数据装入
				My_STMFLASH_Write();//写入Flash
				USART_RX_STA=0;	
				NVIC_SystemReset();// 复位
			}
		}		
  } 
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 
#endif	

