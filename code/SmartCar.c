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

#include "codetab.h"
#include "LQ12864.h"

sbit LED = P3 ^ 2;  // LED
sbit Buzzer = P5 ^ 4;  // 蜂鸣器  记得用一个三极管驱动哦

bit busy;

typedef char I8;
typedef int I16;
typedef long I32;
typedef unsigned char U8; 

U8 SRCHeader = 0x23;
U8 xdata SRCCID[] = {"SRC00000000000001"};
U8 xdata netConfig[] = "AT+CWJAP=\"Gunter\",\"{qwerty123}\"\r\n\0";
U8 xdata DATA_GET[500]={0};//缓冲区长度

U8 CURRENT_LENGTH=0;

static	 unsigned int   Timer4_Count=1;
static	 unsigned int   Timeout_Count=0;

static unsigned char i;

#define FOSC 11059200L          //系统频率
#define BAUD 115200             //串口波特率

#define S1_S0 0x40              //P_SW1.6
#define S1_S1 0x80              //P_SW1.7


void DELAY_MS(unsigned int timeout);		//@11.0592MHz   1ms
void DELAY_1MS() ;
void UART_TC (unsigned char *str);
void UART_T (unsigned char UART_data); //定义串口发送数据变量
void UART_R();//接受数据
void ConnectServer();//连接服务器
void ReConnectServer();//重启WIFI连接服务器
void USART_Init();
void Device_Init();
void SendAckData(U8 len, unsigned char *RES_DATA);
void ConnectSuccess();
void Timer4Init();
unsigned char CheckBCC(unsigned char len, unsigned char *recv);
void ResponseData(unsigned char len,unsigned char *RES_DATA);
void Buzzer_Actions_Status(unsigned char status);
void Led_Actions_Status(unsigned char status);
void Timer0Init(void);
void LEDFunc(void);


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
		
		Device_Init();//初始化硬件


	  OLED_Init(); //OLED初始化
		
		LEDFunc();


    USART_Init();//初始化与WiFi通信的串口
		
		if(PCON&0x10){	//如果是硬启动(上电启动)的话，就进行WiFi的第一次初始化操作，若是热启动（复位启动或看门狗启动）的话直接跳过；因为WiFi在第一次初始化的时候，就进行了“ 保存TCP连接到flash，实现上电透传”
			PCON&=0xef;
			ConnectServer();
		}

		

		ConnectSuccess();
		
		Timer4Init();
		Timer0Init();

	  WDT_CONTR = 0x06;       //看门狗定时器溢出时间计算公式: (12 * 32768 * PS) / FOSC (秒)
                            //设置看门狗定时器分频数为32,溢出时间如下:
                            //11.0592M : 1.14s
                            //18.432M  : 0.68s
                            //20M      : 0.63s
    WDT_CONTR |= 0x20;      //启动看门狗  STC单片机的看门狗一旦启动后，就没法关闭

    while(1) {
			WDT_CONTR |= 0x10;  //喂狗程序
			
			if(DHT11_Read_Data(&DATA_Temphui[0],&DATA_Temphui[1])==0)//温湿度检测
			{
				
				 DATA_Temphui[2]=1;	 
			}
		

    };
}

void LEDFunc(void)	{
//		OLED_Fill(0xff); //屏全亮
//		DELAY_MS(2000);
//		OLED_Fill(0x00); //屏全灭
//		DELAY_MS(200);


//		OLED_P16x16Ch(24,0,1);
//		OLED_P16x16Ch(40,0,2);
//		OLED_P16x16Ch(57,0,3);
//		OLED_P16x16Ch(74,0,4);
//		OLED_P16x16Ch(91,0,5);

		for(i=0; i<8; i++)//通过点整显示汉字 -- i表示字表数组的位置
		{
			OLED_P16x16Ch(i*16,0,i);
		 	OLED_P16x16Ch(i*16,2,i+8);
		 	OLED_P16x16Ch(i*16,4,i+16);
		 	OLED_P16x16Ch(i*16,6,i+24);
		}
		DELAY_MS(8000);



		
		for(i=0; i<8; i++)//通过点整显示汉字 -- i表示字表数组的位置
		{
			OLED_P16x16Ch(i*16,0,i);
		 	OLED_P16x16Ch(i*16,2,i+8);
		 	OLED_P16x16Ch(i*16,4,i+16);
		 	OLED_P16x16Ch(i*16,6,i+24);
		}
	//	DELAY_MS(4000);
//		OLED_CLS();//清屏

//		OLED_P8x16Str(0,0,"温度：12℃");//第一行 -- 8x16的显示单元显示ASCII码
//		OLED_P8x16Str(0,2,"OLED Display");
//		OLED_P8x16Str(0,4,"www.heltec.cn");
//		OLED_P6x8Str(0,6,"cn.heltec@gmail.com");
//		OLED_P6x8Str(0,7,"heltec.taobao.com");
//		DELAY_MS(4000);
	//	OLED_CLS();

//		Draw_BMP(0,0,128,8,BMP1);  //图片显示(图片显示慎用，生成的字表较大，会占用较多空间，FLASH空间8K以下慎用)
//		DELAY_MS(8000);
//		Draw_BMP(0,0,128,8,BMP2);
//		DELAY_MS(8000);
	}

unsigned char CheckBCC(unsigned char len, unsigned char *recv){
	  unsigned char bcc = 0x00;
		unsigned char i=0;
    for(i=0;i<len-1;i++)
    {
        bcc^=recv[i];
    };
    return bcc;

}

void ResponseData(unsigned char len,unsigned char *RES_DATA) {
	
	if(len <26){
		return ;
	}


//校验和
	if(CheckBCC(len, RES_DATA) == RES_DATA[len-1]){
	
		 unsigned int dataCmdFlag =(RES_DATA[2] << 8) | RES_DATA[3];         //命令标识
		 unsigned char dataCmdAck = RES_DATA[4];          //应答标识
		 unsigned char j=0;
		 unsigned char dataEncryptFlag = RES_DATA[22];    //加密方式
		 unsigned char dataUintLength = (RES_DATA[23] << 8) | RES_DATA[24];  //数据长度
		 unsigned char xdata  dataTimestamp[6] = {0x00,0x00,0x00,0x00,0x00,0x00};  //时间数据

	 //校验CID是否正确
		 for(j=5;j<22;j++){
			  if(SRCCID[j-5] != RES_DATA[j]){
				 return;
			 }
		 }
		
		 //校验长度是否正确
		 if ((26 + dataUintLength) != len) {
				return ;
		 }
		 
		 Timeout_Count = 0;//将本地的30s重连计数清零
		 
		 //保存时间
		 for(j=0;j<6;j++){
			 dataTimestamp[j] = RES_DATA[25+j];
		 }
		 
		 if(dataCmdFlag == 0x8001){//连接认证
			 
		 }else if(dataCmdFlag ==0x8002){//实时信息主动上报
			 
		 }else if(dataCmdFlag ==0x8003){//补发
			 
		 }else if(dataCmdFlag ==0x8004){//设备登出
			 
		 }else if(dataCmdFlag ==0x8005){//心跳
			 
		 }else if(dataCmdFlag ==0x8006){//远程控制

			 if(RES_DATA[31] == 0x02){//基础数据查询	温度、湿度、灯、喇叭；请见【信息体定义】
					unsigned char  light_status = LED ? 0x02 : 0x01;
					unsigned char buzzy_status = Buzzer ? 0x02 : 0x01;
					unsigned char xdata ds[37] = {0};
					unsigned char dslen =37;
			  	unsigned char j=0;
				
				  ds[0] = 0X23;//数据头
					ds[1] = 0X23;
					ds[2] = 0X10;//命令标识  下发0x8006  对于的上传是0x1006
					ds[3] = 0X06;
					
					if(dataCmdAck == 0xFE){//应答标识
						ds[4] = 0x01;//成功
						
					}
			
				 for(j=0;j<17;j++){//CID赋值
						ds[j+5] = SRCCID[j];
				 }
				ds[22] = 0X01;//不加密
				ds[23] = 0X00;//长度两位 高位00
				ds[24] = 0X0B;//低位0B 一共11位

				ds[25] = 0X14;//年 0x14+2000 = 2020 
				ds[26] = 0X05;//月 
				ds[27] = 0X18;//日 
				ds[28] = 0X15;//时 
				ds[29] = 0X24;//分
				ds[30] = 0X08;//秒
				
				ds[31] = 0X02;//基础查询   编码


					ds[32] = DATA_Temphui[0]; //基础数据4个字节的数据
					ds[33] = DATA_Temphui[1];
					ds[34] = light_status;
					ds[35] = buzzy_status;
					
			
					
				 ds[dslen-1] = CheckBCC(dslen, ds);//计算校验和  放最后一位
						SendAckData(dslen,ds);

				 
				 
			 }else if(RES_DATA[31] == 0x03){//基础控制	灯、喇叭；请见【信息体定义】
				 			 
					 unsigned char light = RES_DATA[32];
					 unsigned char buzzy = RES_DATA[33];
			 
					 if( light==0x02){
							Led_Actions_Status(0);
						}else if( light==0x01){
							Led_Actions_Status(1);
						}
					 
					 if( buzzy==0x02){
							Buzzer_Actions_Status(0);
					 }else if( buzzy==0x01){
							Buzzer_Actions_Status(1);
					 }
					 
					 
					 
					 
					RES_DATA[0] = 0X23;
					RES_DATA[1] = 0X23;
					RES_DATA[2] = 0X10;
					RES_DATA[3] = 0X06;

					if(dataCmdAck == 0xFE){
						RES_DATA[4] = 0x01;//成功
					
					}
					if(dataCmdAck == 0xFE){//应答标识
						RES_DATA[4] = 0x01;//成功
						
					}
			
				 for(j=0;j<17;j++){//CID赋值
						RES_DATA[j+5] = SRCCID[j];
				 }
				RES_DATA[22] = 0X01;//不加密
				RES_DATA[23] = 0X00;//长度两位 高位00
				RES_DATA[24] = 0X09;//低位09 一共9位    6位的时间+1位的命令标识 + 2位的数据

				RES_DATA[25] = 0X14;//年 0x14+2000 = 2020 
				RES_DATA[26] = 0X05;//月 
				RES_DATA[27] = 0X18;//日 
				RES_DATA[28] = 0X15;//时 
				RES_DATA[29] = 0X24;//分
				RES_DATA[30] = 0X08;//秒
				
				RES_DATA[31] = 0X03;//基础控制  灯、喇叭；请见【信息体定义】
				
//				RES_DATA[32] = RES_DATA[32];// 这两位不用改动  
//				RES_DATA[33] = RES_DATA[33];
				
				
						RES_DATA[len-1] = CheckBCC(len, RES_DATA);//这一帧数据 35个字节 len=35
						SendAckData(len,RES_DATA);

			 
			 }else if(RES_DATA[31] == 0x7F){//重启
				 	IAP_CONTR = 0X20;
			 }
			 
			 
			 
		 }
		 
		
	}
	
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
    Buzzer = 0;
}

//初始化完成滴滴两声
void ConnectSuccess(){

	 LED = 1;
	// Buzzer = 1;
	 DELAY_MS(200);
		LED = 0;
	//	Buzzer = 0;
	 DELAY_MS(200);
	  LED = 1;
	//  Buzzer = 1;
	 DELAY_MS(200);
	  LED = 0;
	//  Buzzer = 0;

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
    PS = 1;
		
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
		TL0 = 0x00;		//设置定时初值
	TH0 = 0xDC;		//设置定时初值
	TF0 = 0;		//清除TF0标志
	TR0 = 1;		//定时器0开始计时
	ET0 = 1;	//允许中断
	
	

	 DATA_GET[CURRENT_LENGTH]=SBUF;
	 CURRENT_LENGTH++;
	
	

}



void SendAckData(U8 len, unsigned char *RES_DATA) {
	
		unsigned int i=0;
    for(i=0; i<len; i++)
    {
				 
				SBUF=RES_DATA[i];
				while(!TI);		//检查发送中断标志位
					TI = 0;	
		}
}

//重启 ESP8266WiFi模块
void ReConnectServer() {

    UART_TC("+++\0"); // 退出透传模式
		 DELAY_MS( 1000);
    UART_TC("AT+RST\r\n\0");  // 复位
		
}

//初始化ESP8266WiFi模块，并连接到服务器
void ConnectServer() {

    DELAY_MS( 1000);

    UART_TC("+++\0"); // 退出透传模式
    DELAY_MS( 1000);
		
		UART_TC("AT+CWMODE=1\r\n\0"); // 这是设置STA模式
    DELAY_MS( 2500);
		
    UART_TC("AT+CIPMUX=0\r\n\0");  // 设置单连接模式
    DELAY_MS(1000);

    UART_TC(netConfig);  // 这一步便是连接wifi，延时的时间要长一些，否则会等不到返回的信息。10s
    DELAY_MS(15000);


    UART_TC("AT+CIPSTART=\"TCP\",\"47.104.19.111\",4001\r\n\0");	// 连接到指定TCP服务器192.168.0.2
    DELAY_MS( 5000);

    UART_TC("AT+CIPMODE=1\r\n\0"); // 设置透传模式
    DELAY_MS( 2000);

   UART_TC("AT+SAVETRANSLINK=1,\"47.104.19.111\",4001,\"TCP\"\r\n\0"); // 保存TCP连接到flash，实现上电透传
   DELAY_MS(1000);

    UART_TC("AT+CIPSEND\r\n\0");	 // 进入透传模式 准备模块与电脑进行互传数据
    DELAY_MS( 1000);
		
		

}

void Timer4Init(void)		
{
	//50 毫秒@11.0592MHz
	T4T3M &= 0xDF;		//定时器时钟12T模式
	T4L = 0x00;		//设置定时初值
	T4H = 0x4C;		//设置定时初值
	T4T3M |= 0x80;		//定时器4开始计时
	
		IE2 |= 0x40;		//开定时器4中断
		EA=1; 	//总中断开启
}


//10s中断自动上报信息
void Timer4_interrupt() interrupt 20    //定时中断入口
{
	

		if(Timer4_Count>=200){  //200 * 50ms = 10s
			  	unsigned char j=0;
					U8 xdata RES_DATA[37]= {0};
          unsigned char RES_LEN= 37;
					unsigned char  light_status = LED ? 0x02 : 0x01;
					unsigned char buzzy_status = Buzzer ? 0x02 : 0x01;
					Timer4_Count = 1;

				  RES_DATA[0] = 0X23;//数据头
					RES_DATA[1] = 0X23;
					RES_DATA[2] = 0X10;//命令标识  下发0x8006  对于的上传是0x1006
					RES_DATA[3] = 0X06;
					RES_DATA[4] = 0xFE;//应答标识
						
				 for(j=0;j<17;j++){//CID赋值
						RES_DATA[j+5] = SRCCID[j];
				 }
				 
				RES_DATA[22] = 0X01;//不加密
				RES_DATA[23] = 0X00;//长度两位 高位00
				RES_DATA[24] = 0X0B;//低位0B 一共11位

				RES_DATA[25] = 0X14;//年 0x14+2000 = 2020 
				RES_DATA[26] = 0X05;//月 
				RES_DATA[27] = 0X18;//日 
				RES_DATA[28] = 0X15;//时 
				RES_DATA[29] = 0X24;//分
				RES_DATA[30] = 0X08;//秒
				
				RES_DATA[31] = 0X02;//基础数据上报


//			if(DATA_Temphui[2]==1)
//			{
//					DATA_Temphui[2]=0;//复位将其  用于检测是否收到数据
//			}

			RES_DATA[32] = DATA_Temphui[0];
			RES_DATA[33] = 	DATA_Temphui[1];
			RES_DATA[34] = light_status;
			RES_DATA[35] = buzzy_status,
			RES_DATA[RES_LEN-1] = CheckBCC(RES_LEN, RES_DATA);
					
		  Timeout_Count++;//每加一次加10s
			
			if(Timeout_Count < 3){
				SendAckData(RES_LEN,RES_DATA);
		}else if(Timeout_Count >= 3){//1min 重启机器
				//UART_TC("+++\0"); // 退出透传模式
        
				ReConnectServer();
				Timeout_Count = 0;
			
			}//else 	if(Timeout_Count > 3){
//						Timeout_Count = 0;
//					  UART_TC("AT+RST\r\n\0");  // 复位
//					//	IAP_CONTR = 0X20;
//				}
				
			
		}else{
			
		    Timer4_Count++;
		}
		
}





void Timer0Init(void)		//10毫秒@11.0592MHz
{
	AUXR &= 0x7F;		//定时器时钟12T模式
	TMOD &= 0xF0;		//设置定时器模式
	TMOD |= 0x01;		//设置定时器模式
	TL0 = 0x00;		//设置定时初值
	TH0 = 0xB8;		//设置定时初值
	TF0 = 0;		//清除TF0标志
	TR0 = 1;		//定时器0开始计时
	ET0 = 1;	//允许中断
}



/********************* Timer0中断函数************************/
void timer0_int (void) interrupt 1
{
	TL0 = 0x00;		//设置定时初值
	TH0 = 0xB8;		//设置定时初值
	TF0 = 0;		//清除TF0标志
	TR0 = 0;		//定时器0开始计时
	ET0 = 0;	//允许中断
	
	ResponseData(CURRENT_LENGTH,DATA_GET);		
	CURRENT_LENGTH = 0;
			
}


void Led_Actions_Status(unsigned char status){

	if(status){
		LED = 0;
	}else{
		LED = 1;
	}

}

void Buzzer_Actions_Status(unsigned char status){

	if(status){
		Buzzer = 0;
	}else{
		Buzzer = 1;
	}

}