#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "spi.h"

#include "port.h"
#include "instance.h"
#include "deca_types.h"
#include "deca_spi.h"
#include "device_info.h"
#include "stmflash.h"
#include "dma.h"
//#include <string.h>
#include "hw_config.h"

#define SWS1_SHF_MODE 0x02	//short frame mode (6.81M)
#define SWS1_CH5_MODE 0x04	//channel 5 mode
#define SWS1_ANC_MODE 0x08  //anchor mode
#define SWS1_A1A_MODE 0x10  //anchor/tag address A1
#define SWS1_A2A_MODE 0x20  //anchor/tag address A2
#define SWS1_A3A_MODE 0x40  //anchor/tag address A3
#define SWS1_USB2SPI_MODE 0x78  //USB to SPI mode
#define SWS1_TXSPECT_MODE 0x38  //Continuous TX spectrum mode
                             //"1234567812345678"
#define SOFTWARE_VER_STRING    "Ver.  2.10  TREK" //16 bytes!

uint8 s1switch = 0;
int instance_anchaddr = 0;
int dr_mode = 0;
int chan, tagaddr, ancaddr;
int instance_mode = ANCHOR;

#define LCD_BUFF_LEN (70)
//uint8 dataseq[LCD_BUFF_LEN];
//uint8 dataseq1[LCD_BUFF_LEN];
uint32_t pauseTWRReports  = 0;
//uint32_t printLCDTWRReports  = 0;
uint8_t sendTWRRawReports = 1;

u8 SendBuff[130];
u8 USB_RxBuff[30];

typedef struct
{
    uint8 channel ;
    uint8 prf ;
    uint8 datarate ;
    uint8 preambleCode ;
    uint8 preambleLength ;
    uint8 pacSize ;
    uint8 nsSFD ;
    uint16 sfdTO ;
} chConfig_t ;


//Configuration for DecaRangeRTLS TREK Modes (4 default use cases selected by the switch S1 [2,3] on EVB1000, indexed 0 to 3 )
chConfig_t chConfig[4] ={
                    //mode 1 - S1: 2 off, 3 off
                    {
                        2,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        4,              // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 2 - S1: 2 on, 3 off
                    {
                        2,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        4,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 3 - S1: 2 off, 3 on
                    {
                        5,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        3,              // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 4 - S1: 2 on, 3 on
                    {
                        5,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    }
};

//Slot and Superframe Configuration for DecaRangeRTLS TREK Modes (4 default use cases selected by the switch S1 [2,3] on EVB1000, indexed 0 to 3 )
sfConfig_t sfConfig[4] ={
                    //mode 1 - S1: 2 off, 3 off
					{
						(28), //ms -
						(10),   //thus 10 slots - thus 280ms superframe means 3.57 Hz location rate (10 slots are needed as AtoA ranging takes 30+ ms)
						(10*28), //superframe period
						(10*28), //poll sleep delay
						(20000)
					},
                    //mode 2 - S1: 2 on, 3 off
                    {
                    	(10),   // slot period ms
                    	(10),   // number of slots (only 10 are used) - thus 100 ms superframe means 10 Hz location rate
                        (10*10), // superframe period (100 ms - gives 10 Hz)
                        (10*10), // poll sleep delay (tag sleep time, usually = superframe period)
                        (2500)
                    },
                    //mode 3 - S1: 2 off, 3 on
                    {
						(28),    // slot period ms
						(10),     // thus 10 slots - thus 280ms superframe means 3.57 Hz location rate
						(10*28),  // superframe period
						(10*28),  // poll sleep delay
						(20000)
                    },
                    //mode 4 - S1: 2 on, 3 on
                    {
						(10),   // slot period ms
						(10),   // number of slots (only 10 are used) - thus 100 ms superframe means 10 Hz location rate
						(10*10), // superframe period (100 ms - gives 10 Hz)
						(10*10), // poll sleep (tag sleep time, usually = superframe period)
						(2500) // this is the Poll to Final delay - 2ms (NOTE: if using 6.81 so only 1 frame per ms allowed LDC)
                    }
};
// ======================================================
//
//  Configure instance tag/anchor/etc... addresses
//
void addressconfigure(uint8 s1switch, uint8 mode)
{
    uint16 instAddress ;

    instance_anchaddr = (((s1switch & SWS1_A1A_MODE) << 2) + (s1switch & SWS1_A2A_MODE) + ((s1switch & SWS1_A3A_MODE) >> 2)) >> 4;

    if(mode == ANCHOR)
    {
    	if(instance_anchaddr > 3)
		{
			instAddress = GATEWAY_ANCHOR_ADDR | 0x4 ; //listener
		}
		else
		{
			instAddress = GATEWAY_ANCHOR_ADDR | instance_anchaddr;
		}
	}
    else
    {
    	instAddress = instance_anchaddr;
    }

    instancesetaddresses(instAddress);
}

uint32 inittestapplication(uint8 s1switch);


//returns the use case / operational mode
int decarangingmode(uint8 s1switch)
{
    int mode = 0;

    if(s1switch & SWS1_SHF_MODE)
    {
        mode = 1;
    }

    if(s1switch & SWS1_CH5_MODE)
    {
        mode = mode + 2;
    }

    return mode;
}

uint32 inittestapplication(uint8 s1switch)
{
    uint32 devID ;
    instanceConfig_t instConfig;
    int result;

    SPI_ConfigFastRate(SPI_BaudRatePrescaler_32);  //max SPI before PLLs configured is ~4M

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    devID = instancereaddeviceid() ;
    if(DWT_DEVICE_ID != devID) //if the read of device ID fails, the DW1000 could be asleep
    {
        port_SPIx_clear_chip_select();  //CS low
        Sleep(1);   //200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
        port_SPIx_set_chip_select();  //CS high
        Sleep(7);
        devID = instancereaddeviceid() ;
        // SPI not working or Unsupported Device ID
        if(DWT_DEVICE_ID != devID)
            return(-1) ;
        //clear the sleep bit - so that after the hard reset below the DW does not go into sleep
        dwt_softreset();
    }

    //reset the DW1000 by driving the RSTn line low
    reset_DW1000();

    result = instance_init() ;
    if (0 > result) return(-1) ; // Some failure has occurred

    SPI_ConfigFastRate(SPI_BaudRatePrescaler_4); //increase SPI to max
    devID = instancereaddeviceid() ;

    if (DWT_DEVICE_ID != devID)   // Means it is NOT DW1000 device
    {
        // SPI not working or Unsupported Device ID
        return(-1) ;
    }

    if((s1switch & SWS1_ANC_MODE) == 0)
    {
        instance_mode = TAG;
    }
    else
    {
        instance_mode = ANCHOR;
    }

    addressconfigure(s1switch, instance_mode) ;                            // set up initial payload configuration

    if((instance_mode == ANCHOR) && (instance_anchaddr > 0x3))
    {
    	instance_mode = LISTENER;
    }

    instancesetrole(instance_mode) ;     // Set this instance role

    // get mode selection (index) this has 4 values see chConfig struct initialiser for details.
    dr_mode = decarangingmode(s1switch);

    chan = instConfig.channelNumber = chConfig[dr_mode].channel ;
    instConfig.preambleCode = chConfig[dr_mode].preambleCode ;
    instConfig.pulseRepFreq = chConfig[dr_mode].prf ;
    instConfig.pacSize = chConfig[dr_mode].pacSize ;
    instConfig.nsSFD = chConfig[dr_mode].nsSFD ;
    instConfig.sfdTO = chConfig[dr_mode].sfdTO ;
    instConfig.dataRate = chConfig[dr_mode].datarate ;
    instConfig.preambleLen = chConfig[dr_mode].preambleLength ;

    instance_config(&instConfig, &sfConfig[dr_mode]) ;                  // Set operating channel etc

    return devID;
}
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
void process_dwRSTn_irq(void)
{
    instance_notify_DW1000_inIDLE(1);
}

void process_deca_irq(void)
{
    do{

        instance_process_irq(0);

    }while(port_CheckEXT_IRQ() == 1); //while IRS line active (ARM can only do edge sensitive interrupts)

}



void configure_continuous_txspectrum_mode(uint8 s1switch)
{
//    uint8 command = 0x2 ;  //return cursor home
//    writetoLCD( 1, 0,  &command);
//	sprintf((char*)&dataseq[0], "Continuous TX %s%d", (s1switch & SWS1_SHF_MODE) ? "S" : "L", chan);
//	writetoLCD( 40, 1, dataseq); //send some data
//	memcpy(dataseq, (const uint8 *) "Spectrum Test   ", 16);
//	writetoLCD( 16, 1, dataseq); //send some data

	//configure DW1000 into Continuous TX mode
	instance_starttxtest(0x1000);
	//measure the power
	//Spectrum Analyser set:
	//FREQ to be channel default e.g. 3.9936 GHz for channel 2
	//SPAN to 1GHz
	//SWEEP TIME 1s
	//RBW and VBW 1MHz
	//measure channel power

	//user has to reset the board to exit mode
	while(1)
	{
		Sleep(2);
	}

}
extern uint32 starttime[];
extern int time_idx;
extern instance_data_t instance_data[NUM_INST] ;

uint8 usbVCOMout[LCD_BUFF_LEN*2];


u8 Transfer_Byte(u8 value)
{
	value = (value & 0xaa)>> 1 | (value & 0x55)<<1;
	value = (value & 0xcc)>> 2 | (value & 0x33)<<2;
	value = (value & 0xf0)>> 4 | (value & 0x0f)<<4;
	return value;
}
u8 Check_Switch_Status(u16 value)
{
	u8 temp = 0x00;
	if( ((u8)(value>>8)) != 0xAA )
		return 0;

//	temp = Transfer_Byte((u8)value);
	temp = (u8)value;

	return temp;
}

void Display_SwitchInfo(u8 value)
{

	int n =0;

	printf("Mini3s 配置信息：");
	n = sprintf((char*)&SendBuff[0], "Mini3s 配置信息：");
	if(((value >> 6) & 0x01 ) == 0x01)
	{
		printf("6.8M,  ");
		n+=	sprintf((char*)&SendBuff[n], "6.8M,  ");
	}
	else
	{
		printf("110kpbs,  ");
		n+=	sprintf((char*)&SendBuff[n], "110kpbs,  ");
	}

	if(((value >> 5) & 0x01 ) == 0x01)
	{
		printf("Channel 5, ");
		n+=	sprintf((char*)&SendBuff[n], "Channel 5, ");
	}
	else
	{
		printf("Channel 2, ");
		n+=	sprintf((char*)&SendBuff[n], "Channel 2, ");
	}

	if(((value >> 4) & 0x01 ) == 0x01)
	{
		printf("Anchor %d\r\n", (value >> 1) & 0x07  );
		n+=	sprintf((char*)&SendBuff[n], "Anchor %d\r\n",(value >> 1) & 0x07);
	}
	else
	{
		printf("Tag %d\r\n", (value >> 1) & 0x07 );
		n+=	sprintf((char*)&SendBuff[n], "Tag %d\r\n",(value >> 1) & 0x07);
	}


	USB_TxWrite(SendBuff, n);

}


//const u8 TEXT_TO_SEND[]={"ALIENTEK Warship STM32 DMA 串口实验"};

int main(void)
{
	int i = 0;
	int rx = 0;
	int toggle = 0;

	u8 t=0;
	u8 j,mask=0;

	u8 len;
	u8 temp_switch=0;
	u16 times=0;
  int n=0;

  //delay_init();	    	 //延时函数初始化
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);	 //串口初始化为115200
	GPIO_Configuration();//初始化与LED连接的硬件接口
	SPI_Configuration();
	peripherals_init();

	Sleep(1000);
	Flash_Configuration();
	Sleep(200);
	USB_Config();
 	MYDMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)SendBuff,130);//DMA1通道4,外设为串口1,存储器为SendBuff,长度130.

	printf("|*******************(C) COPYRIGHT 2017 研创物联*******************\r\n");
	printf("|                                                                 \r\n");
	printf("|    UWB Mini3s - Real Time Location System (Version 20170517)   \r\n");
	printf("|            By Vincent Lynn from YanChuang IOT ltd               \r\n");
	printf("|                                                                 \r\n");
	printf("|*******************(C) COPYRIGHT 2017 研创物联*******************\r\n");
	printf("Mini3s 开机次数：%d\r\n"    ,sys_para.start_count);

	n = sprintf((char*)&SendBuff[0], "|*******************(C) COPYRIGHT 2017 研创物联*******************\r\n");
	n+=	sprintf((char*)&SendBuff[n], "|                                                                 \r\n");
	n+=	sprintf((char*)&SendBuff[n], "|    UWB Mini3s - Real Time Location System (Version 20170517)   \r\n");
	n+=	sprintf((char*)&SendBuff[n], "|            By Vincent Lynn from YanChuang IOT ltd               \r\n");
	n+=	sprintf((char*)&SendBuff[n], "|                                                                 \r\n");
	n+=	sprintf((char*)&SendBuff[n], "|*******************(C) COPYRIGHT 2017 研创物联*******************\r\n");
	USB_TxWrite(SendBuff, n);




	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); //使能串口1的DMA发送
	MYDMA_Enable(DMA1_Channel4);//开始一次DMA传输！

	temp_switch = Check_Switch_Status(sys_para.device_switch);//存按拨码开关正常顺序存
	printf("sys_para.device_switch =%02x\r\n",sys_para.device_switch);
	//如果值是错的，继续等待，灯闪烁
	while( temp_switch == 0x00 )
	{
		i=5;
		while(i--)
		{
				if (i & 1) led_off(LED_ALL);
				else    led_on(LED_ALL);

				Sleep(200);
		}
		i = 0;
		printf("等待接收AT命令进行虚拟拨码开关设置\r\n");


		//usb等待接收 AT+SW序列
		len = USB_RxRead(USB_RxBuff, sizeof(USB_RxBuff));
		if(Check_cmd(USB_RxBuff)==SW)
		{
				Sleep(200);
				FLAH_BUFF0[2] = GetRecSwtich(USB_RxBuff);//数据装入
				My_STMFLASH_Write();//写入Flash
				NVIC_SystemReset();// 复位
		}

		USB_TxWrite("等待接收AT命令进行虚拟拨码开关设置\r\n", 38);
	}



	Display_SwitchInfo(temp_switch);
	s1switch = Transfer_Byte(temp_switch & 0x7f);
	printf("s1switch =%02x\r\n",s1switch);
	port_DisableEXT_IRQ(); //disable ScenSor IRQ until we configure the device
	led_off(LED_ALL);

	if(inittestapplication(s1switch) == (uint32)-1)
	{
		led_on(LED_ALL); //to display error....
		return 0; //error
	}
	i=50;
	while(i--)
	{
				if (i & 1) led_off(LED_ALL);
				else    led_on(LED_ALL);

				Sleep(50);
	}
	i = 0;

	// Is continuous spectrum test mode selected?
	if((s1switch & SWS1_TXSPECT_MODE) == SWS1_TXSPECT_MODE)
	{
		//this function does not return!
		configure_continuous_txspectrum_mode(s1switch);
	}

		//sleep for 5 seconds displaying last LCD message and flashing LEDs

		led_off(LED_ALL);
    port_EnableEXT_IRQ(); //enable ScenSor IRQ before starting
//    memset(dataseq1, ' ', LCD_BUFF_LEN);
//
 	while(1)
	{


    	int monitor_local = instance_data[0].monitor ;
    	int txdiff = (portGetTickCnt() - instance_data[0].timeofTx);
			n = 0;	//sprintf count
        instance_run();
        instance_mode = instancegetrole();

        //if delayed TX scheduled but did not happen after expected time then it has failed... (has to be < slot period)
        //if anchor just go into RX and wait for next message from tags/anchors
        //if tag handle as a timeout
        if((monitor_local == 1) && ( txdiff > instance_data[0].slotPeriod))
        {
        	int an = 0;
        	uint32 tdly ;
        	uint32 reg1, reg2;

        	reg1 = dwt_read32bitoffsetreg(0x0f, 0x1);
        	reg2 = dwt_read32bitoffsetreg(0x019, 0x1);
        	tdly = dwt_read32bitoffsetreg(0x0a, 0x1);
//        	an = sprintf((char*)&usbVCOMout[0], "T%08x %08x time %08x %08x", (unsigned int) reg2, (unsigned int) reg1,
//        			(unsigned int) dwt_read32bitoffsetreg(0x06, 0x1), (unsigned int) tdly);
//            send_usbmessage(&usbVCOMout[0], an);

					instance_data[0].wait4ack = 0;

					if(instance_mode == TAG)
					{
						inst_processrxtimeout(&instance_data[0]);
					}
					else //if(instance_mode == ANCHOR)
					{
						dwt_forcetrxoff();	//this will clear all events
						//dwt_rxreset();
						//enable the RX
						instance_data[0].testAppState = TA_RXE_WAIT ;
					}
					instance_data[0].monitor = 0;
        }

        rx = instancenewrange();

        //if there is a new ranging report received or a new range has been calculated, then prepare data
        //to output over USB - Virtual COM port, and update the LCD
        if(rx != TOF_REPORT_NUL)
        {
        	int l = 0, r= 0, aaddr, taddr;
        	int rangeTime, valid;
        	//int correction ;
            uint16 txa, rxa;

						MYDMA_Enable(DMA1_Channel4);//开始一次DMA传输！
//				if(DMA_GetFlagStatus(DMA1_FLAG_TC4)!=RESET)	//判断通道4传输完成
//				{
//					DMA_ClearFlag(DMA1_FLAG_TC4);//清除通道4传输完成标志
//					break;
//		        }
            //send the new range information to LCD and/or USB
            aaddr = instancenewrangeancadd() & 0xf;
            taddr = instancenewrangetagadd() & 0xf;
            rangeTime = instancenewrangetim() & 0xffffffff;

            l = instancegetlcount() & 0xFFFF;
            if(instance_mode == TAG)
            {
            	r = instancegetrnum();
            }
            else
            {
            	r = instancegetrnuma(taddr);
            }
            txa =  instancetxantdly();
            rxa =  instancerxantdly();
            valid = instancevalidranges();

            n = 0;
            if(rx == TOF_REPORT_T2A)
            {
            	//correction = instance_data[0].tagSleepCorrection2;
            	// anchorID tagID range rangeraw countofranges rangenum rangetime txantdly rxantdly address
            	n = sprintf((char*)&SendBuff[0], "mc %02x %08x %08x %08x %08x %04x %02x %08x %c%d:%d\r\n",
														valid, instancegetidist_mm(0), instancegetidist_mm(1),
														instancegetidist_mm(2), instancegetidist_mm(3),
														l, r, rangeTime,
														(instance_mode == TAG)?'t':'a', taddr, aaddr);

//							USB_TxWrite(SendBuff, n);
//							printf("mc %02x %08x %08x %08x %08x %04x %02x %08x %c%d:%d\r\n",
//							            valid, instancegetidist_mm(0), instancegetidist_mm(1),
//														instancegetidist_mm(2), instancegetidist_mm(3),
//														l, r, rangeTime,
//														(instance_mode == TAG)?'t':'a', taddr, aaddr);

            	if(sendTWRRawReports == 1)
            	{
            		n += sprintf((char*)&SendBuff[n], "mr %02x %08x %08x %08x %08x %04x %02x %04x%04x %c%d:%d\r\n",
														valid, instancegetidistraw_mm(0), instancegetidistraw_mm(1),
														instancegetidistraw_mm(2), instancegetidistraw_mm(3),
														l, r, txa, rxa,
														(instance_mode == TAG)?'t':'a', taddr, aaddr);
//								printf("mr %02x %08x %08x %08x %08x %04x %02x %04x%04x %c%d:%d\r\n",
//														valid, instancegetidistraw_mm(0), instancegetidistraw_mm(1),
//														instancegetidistraw_mm(2), instancegetidistraw_mm(3),
//														l, r, txa, rxa,
//														(instance_mode == TAG)?'t':'a', taddr, aaddr);

            	}
//							USB_TxWrite(SendBuff, 130);
//							memcpy((u8*)SendBuff, (u8*)usbVCOMout, 65);
            }
            else //anchor to anchor ranging
            {
            	n = sprintf((char*)&SendBuff[0], "ma %02x %08x %08x %08x %08x %04x %02x %08x a0:%d\r\n",
														valid, instancegetidist_mm(0), instancegetidist_mm(1),
														instancegetidist_mm(2), instancegetidist_mm(3),
														l, instancegetrnumanc(0), rangeTime, aaddr);
//							printf("ma %02x %08x %08x %08x %08x %04x %02x %08x a0:%d\r\n",
//														valid, instancegetidist_mm(0), instancegetidist_mm(1),
//														instancegetidist_mm(2), instancegetidist_mm(3),
//														l, instancegetrnumanc(0), rangeTime, aaddr);
//							USB_TxWrite(SendBuff, 65);
            }
            instancecleardisttableall();
        }

				if(n > 0)
       	{
       		if(pauseTWRReports == 0)
       		{
							USB_TxWrite(SendBuff, n);
       		}
       		else
       		{
       			if(pauseTWRReports + 1000 <= portGetTickCnt())
       			{
       				pauseTWRReports = 0;
       			}
       		}
       	}

				len = USB_RxRead(USB_RxBuff, sizeof(USB_RxBuff));
				if(Check_cmd(USB_RxBuff)==SW)
				{
//					Display_SwitchInfo(temp_switch);
					Sleep(200);
					FLAH_BUFF0[2] = GetRecSwtich(USB_RxBuff);//数据装入
					My_STMFLASH_Write();//写入Flash
					NVIC_SystemReset();// 复位
				}
				if(Check_cmd(USB_RxBuff)==DECA)
				{
					USB_TxWrite(buff_version, 19);
				}

	}

}

