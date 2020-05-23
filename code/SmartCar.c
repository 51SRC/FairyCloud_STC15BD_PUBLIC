/*---------------------------------------------------------------------*/
/* --- STC MCU Limited ------------------------------------------------*/
/* --- STC15F4K60S4 系列 定时器1用作串口1的波特率发生器举例------------*/
/* --- Mobile: (86)13922805190 ----------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ------------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966-------------------------*/
/* --- Web: www.STCMCU.com --------------------------------------------*/
/* --- Web: www.GXWMCU.com --------------------------------------------*/
/* 如果要在程序中使用此代码,请在程序中注明使用了STC的资料及程序        */
/* 如果要在文章中应用此代码,请在文章中注明使用了STC的资料及程序        */
/*---------------------------------------------------------------------*/

//本示例在Keil开发环境下请选择Intel的8058芯片型号进行编译
//若无特别说明,工作频率一般为11.0592MHz


#include "STC15W4K58S4.h"
#include "DHT11.h"

#include "intrins.h"
#include <string.h>  // 字符串处理头文件

sbit LED = P3 ^ 2;  // 对应硬件连接
sbit LOUND = P5 ^ 4;  // 对应硬件连接

bit busy;

typedef char I8;
typedef int I16;
typedef long I32;
typedef unsigned char U8; 

U8 DATA_LENGTH = 9;
U8 DATA_GET[]=  { 0x7E, 0, 0, 0, 0, 0, 0, 0, 0x7E};
U8 SRCHeader = 0x7E;
U8 SRCTail = 0x7E;
U8 SRCDeviceID = 0x03;
U8 SRCAID = 0x01;
U8 CURRENT_LENGTH=0;
U8 go=0;
static	 unsigned int   Timer4_Count=1;
unsigned char RES_DATA[]= { 0x7E, 0x00,0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x7E};



#define FOSC 11059200L          //系统频率
#define BAUD 115200             //串口波特率

#define S1_S0 0x40              //P_SW1.6
#define S1_S1 0x80              //P_SW1.7


void SendString(char *s);
void SendDatas(char *s);
void DELAY_MS(unsigned int timeout);		//@11.0592MHz   1ms
void DELAY_1MS() ;
void SendData(char *s);
void UART_TC (unsigned char *str);
void UART_T (unsigned char UART_data); //定义串口发送数据变量
void UART_R();//接受数据
void ConnectServer();//连接服务器
void USART_Init();
void Device_Init();
void ResponseData(unsigned char *RES_DATA);
unsigned char CheckData(unsigned char *CHECK_DATA);
void sendAckData(unsigned char *RES_DATA);
void ConnectSuccess();
void Timer4Init();


void main(){
		P0M0 = 0x00;
    P0M1 = 0x00;
    P1M0 = 0x00;
    P1M1 = 0x00;
    P2M0 = 0x00;
    P2M1 = 0x00;
    P3M0 = 0x00;
    P3M1 = 0x00;
    P4M0 = 0x00;
    P4M1 = 0x00;
    P5M0 = 0x00;
    P5M1 = 0x00;
    P6M0 = 0x00;
    P6M1 = 0x00;
    P7M0 = 0x00;
    P7M1 = 0x00;

    Device_Init();

    USART_Init();

		ConnectServer();

		ConnectSuccess();
		
		Timer4Init();
    while(1) {
			
			if(DHT11_Read_Data(&DATA_Temphui[0],&DATA_Temphui[1])==0)//温湿度检测
			{
				
				 DATA_Temphui[2]=1;	 
			}
		

    };
}

void DELAY_1MS() {
    unsigned char i, j;

    _nop_();
    _nop_();
    _nop_();
    i = 11;
    j = 190;
    do
    {
        while (--j);
    } while (--i);


}

void DELAY_MS(unsigned int timeout)		//@11.0592MHz
{
    int t = 0;
    while (t < timeout)
    {
        t++;
        DELAY_1MS();
    }
}



//初始化LED和蜂鸣器
void Device_Init() {

    LED = 0;
    LOUND = 0;
}

//初始化完成滴滴两声
void ConnectSuccess(){

	 LED = 1;
	 LOUND = 1;
	 DELAY_MS(200);
		LED = 0;
		LOUND = 0;
	 DELAY_MS(200);
	  LED = 1;
	  LOUND = 1;
	 DELAY_MS(200);
	  LED = 0;
	  LOUND = 0;

}


void USART_Init()
{

//   ACC = P_SW1;
//    ACC &= ~(S1_S0 | S1_S1);    //S1_S0=0 S1_S1=0
//    P_SW1 = ACC;                //(P3.0/RxD, P3.1/TxD)

    ACC = P_SW1;
    ACC &= ~(S1_S0 | S1_S1);    //S1_S0=1 S1_S1=0
    ACC |= S1_S0;               //(P3.6/RxD_2, P3.7/TxD_2)
    P_SW1 = ACC;
    SCON = 0x50;                //8位可变波特率

//  ACC = P_SW1;
//  ACC &= ~(S1_S0 | S1_S1);    //S1_S0=0 S1_S1=1
//  ACC |= S1_S1;               //(P1.6/RxD_3, P1.7/TxD_3)
//  P_SW1 = ACC;


    AUXR = 0x40;                //定时器1为1T模式
    TMOD = 0x00;                //定时器1为模式0(16位自动重载)
    TL1 = (65536 - (FOSC/4/BAUD));   //设置波特率重装值
    TH1 = (65536 - (FOSC/4/BAUD))>>8;
    TR1 = 1;                    //定时器1开始启动
    ES = 1;                     //使能串口中断
    EA = 1;

}

/*----------------------------
UART 中断服务程序
-----------------------------*/
void Uart() interrupt 4 using 1
{
    if (RI)
    {
        while(!RI);
        RI=0;
        UART_R();
        busy = 0;

    }
    if (TI)
    {
        while(!TI);
        TI = 0;                 //清除TI位
        busy = 0;               //清忙标志
    }
}


/*----------------------------
发送串口数据
----------------------------*/

void  SendData(char *s)
{
    unsigned int i=0;

    for(i=0; i<DATA_LENGTH; i++)
    {
				 
        SBUF=s[i];
				 while(!TI);		//检查发送中断标志位
				TI = 0;	
				}
}

void UART_T (unsigned char UART_data) { //定义串口发送数据变量
    SBUF = UART_data;	//将接收的数据发送回去
    while(!TI);		//检查发送中断标志位
    TI = 0;			//令发送中断标志位为0（软件清零）
}


void UART_TC (unsigned char *str) {
    while(*str != '\0') {
        UART_T(*str);
        *str++;
    }
    *str = 0;
}


//串口  接收ESP8266的串口数据，并校验数据的完整性9位

void UART_R()
{
		if((CURRENT_LENGTH==0)&&(SBUF==0x7E))// 判断第一个是不是0x7e  不确定是不是尾部7e
	   {go=1;}

			if((go==1)&&(CURRENT_LENGTH==1)&&(SBUF==0X7E))//第二个7e
				 {CURRENT_LENGTH=0;} 

			if(go==1)  //9个字符可以运行了
			{
				 DATA_GET[CURRENT_LENGTH]=SBUF;
				CURRENT_LENGTH++;
				
				
				if(CURRENT_LENGTH==DATA_LENGTH)
				{
						CURRENT_LENGTH=0;
						go = 0;
						ResponseData(DATA_GET);
				}
			}

}



///校验数据准确性 做CRC校验
unsigned char CheckData(unsigned char *mes){
    unsigned char crc = 0;
    unsigned char len = 6;
    unsigned char i=0;
    unsigned char cs=0;
    unsigned char message[] = {0,0,0,0,0,0};
    unsigned char *s = message;
    for( cs=0;cs<len;cs++){
        
        s[cs] = mes[cs+1];
    }
    
    
    while(len--)
    {
        crc ^= *s++;
        for(i = 0;i < 8;i++)
        {
            if(crc & 0x01)
            {
                crc = (crc >> 1) ^ 0x8c;
            }
            else crc >>= 1;
        }
    }
    return crc;
}

//处理接收到的数据
void ResponseData(unsigned char *RES_DATA) {

    if((CheckData(RES_DATA) == RES_DATA[DATA_LENGTH-2]) && RES_DATA[1]== SRCDeviceID &&  RES_DATA[2]== SRCAID && RES_DATA[4]== 0x01 ) {

        if(RES_DATA[3]==0x00 ) {
						if(DATA_Temphui[2]==1){
									RES_DATA[3]=0x04;//高两位数据 4代表温湿度指令
									RES_DATA[5]= DATA_Temphui[0];//高两位数据
									RES_DATA[6]= DATA_Temphui[1];//进制转换  低两位数据位
								//	DATA_Temphui[2]=0;
						}	
            sendAckData(RES_DATA);
        } else	if(RES_DATA[3]==0x03 && RES_DATA[6]==0x02) {
            LED = 1;
            sendAckData(RES_DATA);
        } else	if(RES_DATA[3]==0x03 && RES_DATA[6]==0x01) {
            LED = 0;
            sendAckData(RES_DATA);
        } else if(RES_DATA[3]==0x02 && RES_DATA[6]==0x02) {
            LOUND = 1;
            sendAckData(RES_DATA);
        } else	if(RES_DATA[3]==0x02 && RES_DATA[6]==0x01) {
            LOUND = 0;
            sendAckData(RES_DATA);
        }
    }

}

//处理待发送的数据
void sendAckData(unsigned char *RES_DATA) {


    unsigned char DATA_SEND[]= { 0x7E, 0x00,0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x7E};


    DATA_SEND[0]= SRCHeader;
    DATA_SEND[1]= SRCDeviceID;
    DATA_SEND[2]= SRCAID;
    DATA_SEND[3]= RES_DATA[3];
    DATA_SEND[5]= RES_DATA[5];
    DATA_SEND[6]= RES_DATA[6];
		DATA_SEND[7]= CheckData(DATA_SEND);

    DATA_SEND[DATA_LENGTH-1]= SRCTail;
		
    SendData(DATA_SEND);

}

//初始化ESP8266WiFi模块，并连接到服务器
void ConnectServer() {

    DELAY_MS( 1000);

    UART_TC("+++\0"); // 退出透传模式
    DELAY_MS( 1000);

//    UART_TC("AT+RST\r\n\0");  // 复位
//    DELAY_MS(2000);
		
		UART_TC("AT+CWMODE=1\r\n\0"); // 这是设置STA模式
    DELAY_MS( 2500);
		
    UART_TC("AT+CIPMUX=0\r\n\0");  // 设置单连接模式
    DELAY_MS(1000);

    UART_TC("AT+CWJAP=\"Gunter\",\"{qwerty123}\"\r\n\0");  // 这一步便是连接wifi，延时的时间要长一些，否则会等不到返回的信息。10s
    DELAY_MS(10000);


    UART_TC("AT+CIPSTART=\"TCP\",\"47.104.19.111\",4001\r\n\0");	// 连接到指定TCP服务器
    DELAY_MS( 5000);

    UART_TC("AT+CIPMODE=1\r\n\0"); // 设置透传模式
    DELAY_MS( 2000);

  // UART_TC("AT+SAVETRANSLINK=1,\"47.104.19.111\",4001,\"TCP\"\r\n\0"); // 保存TCP连接到flash，实现上电透传
  // DELAY_MS(1000);

    UART_TC("AT+CIPSEND\r\n\0");	 // 进入透传模式 准备模块与电脑进行互传数据
    DELAY_MS( 1000);

    CURRENT_LENGTH=0;
		
		

}

void Timer4Init(void)		//5毫秒@11.0592MHz
{
	T4T3M |= 0x20;		//定时器时钟1T模式
	T4L = 0x00;		//设置定时初值
	T4H = 0x28;		//设置定时初值
	T4T3M |= 0x80;		//定时器4开始计时
		IE2 |= 0x40;		//开定时器4中断
		EA=1; 	//总中断开启
}


//10s自动上报温湿度
void Timer4_interrupt() interrupt 20    //定时中断入口
{
		if(Timer4_Count>=2000){
			Timer4_Count = 1;
			
			if(DATA_Temphui[2]==1){
						RES_DATA[3]=0x04;//高两位数据 4代表温湿度指令
						RES_DATA[5]= DATA_Temphui[0];//高两位数据
						RES_DATA[6]= DATA_Temphui[1];//进制转换  低两位数据位
					//	DATA_Temphui[2]=0;
			}	
			
			sendAckData(RES_DATA);
		}else{
			
			Timer4_Count++;
		}
}



//void Timer0Init(void)		//10毫秒@11.0592MHz
//{
//	AUXR &= 0x7F;		//定时器时钟12T模式
//	TMOD &= 0xF0;		//设置定时器模式
//	TMOD |= 0x01;		//设置定时器模式
//	TL0 = 0x00;		//设置定时初值
//	TH0 = 0xDC;		//设置定时初值
//	TF0 = 0;		//清除TF0标志
//	TR0 = 1;		//定时器0开始计时
//	ET0 = 1;	//允许中断
//}

///********************* Timer0中断函数************************/
//void timer0_int (void) interrupt 1
//{
//   
//}
