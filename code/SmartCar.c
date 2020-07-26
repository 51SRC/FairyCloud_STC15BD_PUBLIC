/*---------------------------------------------------------------------*/
/* --- STC MCU Limited ------------------------------------------------*/
/* --- STC15F4K60S4 ϵ�� ��ʱ��1��������1�Ĳ����ʷ���������------------*/
/* --- Mobile: (86)13922805190 ----------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ------------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966-------------------------*/
/* --- Web: www.STCMCU.com --------------------------------------------*/
/* --- Web: www.GXWMCU.com --------------------------------------------*/
/* ���Ҫ�ڳ�����ʹ�ô˴���,���ڳ�����ע��ʹ����STC�����ϼ�����        */
/* ���Ҫ��������Ӧ�ô˴���,����������ע��ʹ����STC�����ϼ�����        */
/*---------------------------------------------------------------------*/




/* --------------------------- ��Ҫ˵�� -------------------------------------------------------------------------------*/

/*--1.΢�Ŷ��ĺ����¿��ܲ������µģ�����������µ�---------------------------------------------------------------------*/
/*--2.����������һ��ˮ��PUMP�����ڵ�������P2^3��P2^3��LED��------------------------------------------------------------*/
/*--3.12864��ʾ�������Ŷ�Ӧ��ϵ //sbit SCL=P4^1; ����ʱ��  //sbit SDA=P4^2; ��������---��ͷ�ļ�L112864,H���޸�---------*/
/*--4.��Ƭ����������Ƭ������==����ȡ������ʱ��==��OPENID��֤==����������==������ͨ��-----------------------------------*/
/*--5.�����룬�����SRCCIDΪ����豸CID,  SRCOPENIDΪ���openid ,      netConfig������������WiFi��������-------------*/
/* --------------------------- QQ:1055417926 --------------------------------------------------------------------------*/

/* --------------------------- END ------------------------------------------------------------------------------------*/


#include "STC15W4K58S4.h"
#include "DHT11.h"
#include "intrins.h"
#include <string.h>  // �ַ�������ͷ�ļ�

#include "codetab.h"
#include "LQ12864.h"

sbit LED = P3 ^ 2;  // LED
sbit PUMP = P3 ^ 2;  
//sbit PUMP = P2 ^ 3;  // ˮ��PUMP
sbit Buzzer = P5 ^ 4;  // ������  �ǵ���һ������������Ŷ
//sbit SCL=P4^1; //����ʱ��
//sbit SDA=P4^2; //��������

bit busy;

typedef char I8;
typedef int I16;
typedef long I32;
typedef unsigned char U8; 

U8 SRCHeader = 0x23;
U8 xdata SRCCID[] = {"SRC00000000000004"};//����豸ID 17λ
U8 xdata netConfig[] = "AT+CWJAP=\"Gunter\",\"{qwerty123}\"\r\n\0";// ���WiFi���ƺ�����  ʾ����WiFi���� Gunter     ����    {qwerty123}
U8 xdata DATA_GET[500]={0};//����������


U8 xdata SRCOPENID[] = {"627460345c74ffa8d078c98b541fb092"};//���32λ��openid������С���򡶺��Ǿ��顷�������Ĳ鿴
static	  unsigned char   CheckTime = 0;  //�Ƿ��Ѻͷ�����ͬ��ʱ��    ��ʶ
static	  unsigned char   CheckAuth = 0; //�Ƿ��ѵ�½��֤   ��ʶ
static	 unsigned int xdata  Timestamp[6] = {0x00,0x00,0x00,0x00,0x00,0x00};  //���������������ʱ������



U8 CURRENT_LENGTH=0;

static	 unsigned int   Timer4_Count=1;
static	 unsigned int   Timeout_Count=0;
static	 unsigned int   DisplayTime_Count=0;
static   unsigned char i;

#define FOSC 11059200L          //ϵͳƵ��
#define BAUD 115200             //���ڲ�����

#define S1_S0 0x40              //P_SW1.6
#define S1_S1 0x80              //P_SW1.7


void DELAY_MS(unsigned int timeout);		//@11.0592MHz   1ms
void DELAY_1MS() ;
void UART_TC (unsigned char *str);
void UART_T (unsigned char UART_data); //���崮�ڷ������ݱ���
void UART_R();//��������
void ConnectServer();//���ӷ�����
void ReConnectServer();//����WIFI���ӷ�����
void USART_Init();
void Device_Init();
void SendAckData(U8 len, unsigned char *RES_DATA);
void ConnectSuccess();
void Timer4Init();
unsigned char CheckBCC(unsigned char len, unsigned char *recv);
void ResponseData(unsigned char len,unsigned char *RES_DATA);
void Buzzer_Actions_Status(unsigned char status);
void Led_Actions_Status(unsigned char status);
void Pump_Actions_Status(unsigned char status);//����ˮ��

void Timer0Init(void);
void LEDFunc(unsigned char TEMP,unsigned char HUMI)	;//LED��ʾ��ʪ��// SCL��P4^1  // SDA��P4^2


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
		
		Device_Init();//��ʼ��Ӳ��


	  OLED_Init(); //OLED��ʼ��
		
		LEDFunc(0,0);
		OLED_P6x8Str(0,7,"status: Starting    ");//connected closed starting


    USART_Init();//��ʼ����WiFiͨ�ŵĴ���
		
		if(PCON&0x10){	//�����Ӳ����(�ϵ�����)�Ļ����ͽ���WiFi�ĵ�һ�γ�ʼ����������������������λ�������Ź��������Ļ�ֱ����������ΪWiFi�ڵ�һ�γ�ʼ����ʱ�򣬾ͽ����ˡ� ����TCP���ӵ�flash��ʵ���ϵ�͸����
			PCON&=0xef;
			ConnectServer();
		}

		

		ConnectSuccess();
		
		Timer4Init();
		Timer0Init();

	  WDT_CONTR = 0x06;       //���Ź���ʱ�����ʱ����㹫ʽ: (12 * 32768 * PS) / FOSC (��)
                            //���ÿ��Ź���ʱ����Ƶ��Ϊ32,���ʱ������:
                            //11.0592M : 1.14s
                            //18.432M  : 0.68s
                            //20M      : 0.63s
    WDT_CONTR |= 0x20;      //�������Ź�  STC��Ƭ���Ŀ��Ź�һ�������󣬾�û���ر�

    while(1) {
			WDT_CONTR |= 0x10;  //ι������
			
			if(DHT11_Read_Data(&DATA_Temphui[0],&DATA_Temphui[1])==0)//��ʪ�ȼ��
			{
				
				  DATA_Temphui[2]=1;	 
					LEDFunc(DATA_Temphui[0],DATA_Temphui[1]);

			}
		

    };
}

void LEDFunc(unsigned char TEMP,unsigned char HUMI)	{
	
	
		OLED_P16x16Ch(0,4,16);//ˮ��
		OLED_P16x16Ch(16,4,17);
		OLED_P16x16Ch(72,4,18);//����
		OLED_P16x16Ch(88,4,19);
		
	//	OLED_P8x16Str(0,0,"00.00   00:00:00");//��ʾʱ��
		
		
		
		OLED_P8x16Char(0,0,'0'+Timestamp[1]/10%10); //��
	  OLED_P8x16Char(8,0,'0'+Timestamp[1]%10);		
		OLED_P8x16Str(16,0,".");
		
		OLED_P8x16Char(24,0,'0'+Timestamp[2]/10%10); //��
	  OLED_P8x16Char(32,0,'0'+Timestamp[2]%10);		
		OLED_P8x16Str(40,0,"   ");
		
		OLED_P8x16Char(64,0,'0'+Timestamp[3]/10%10); //ʱ
	  OLED_P8x16Char(72,0,'0'+Timestamp[3]%10);		
		OLED_P8x16Str(80,0,":");
		
		OLED_P8x16Char(88,0,'0'+Timestamp[4]/10%10); //��
	  OLED_P8x16Char(96,0,'0'+Timestamp[4]%10);		
		OLED_P8x16Str(104,0,":");
		
		OLED_P8x16Char(112,0,'0'+Timestamp[5]/10%10); //��
	  OLED_P8x16Char(120,0,'0'+Timestamp[5]%10);		
		



		OLED_P8x16Str(0,2,"Temp:");
		OLED_P8x16Str(72,2,"Humi:");


	  OLED_P8x16Char(40,2,'0'+TEMP/10%10);
	  OLED_P8x16Char(48,2,'0'+TEMP%10);		
		OLED_P8x16Char(112,2,'0'+HUMI/10%10);
	  OLED_P8x16Char(120,2,'0'+HUMI%10);


		OLED_P8x16Str(32,4,":");
		OLED_P8x16Str(104,4,":");
		
	  if(PUMP){//��ʾˮ��״̬ ��/��
					OLED_P16x16Ch(40,4,24);

		}else{
					OLED_P16x16Ch(40,4,25);

		}

		if(Buzzer){//��ʾ������״̬ ��/��
					OLED_P16x16Ch(112,4,24);

		}else{
					OLED_P16x16Ch(112,4,25);

		}
		
				
		if(CheckTime==0){
				OLED_P6x8Str(0,7,"status: CheckTime  ");//connected closed starting
		}else if(CheckAuth==0){
				OLED_P6x8Str(0,7,"status: CheckAuth  ");//connected closed starting
		}else{
			  OLED_P6x8Str(0,7,"status: Connected  ");//Connected  Starting
		}
				

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


//У���
	if(CheckBCC(len, RES_DATA) == RES_DATA[len-1]){
	
		 unsigned int dataCmdFlag =(RES_DATA[2] << 8) | RES_DATA[3];         //�����ʶ
		 unsigned char dataCmdAck = RES_DATA[4];          //Ӧ���ʶ
		 unsigned char j=0;
		 unsigned char dataEncryptFlag = RES_DATA[22];    //���ܷ�ʽ
		 unsigned char dataUintLength = (RES_DATA[23] << 8) | RES_DATA[24];  //���ݳ���

	 //У��CID�Ƿ���ȷ
		 for(j=5;j<22;j++){
			  if(SRCCID[j-5] != RES_DATA[j]){
				 return;
			 }
		 }
		
		 //У�鳤���Ƿ���ȷ
		 if ((26 + dataUintLength) != len) {
				return ;
		 }
		 
		 Timeout_Count = 0;//�����ص�30s������������
		 

		 
		 if(dataCmdFlag == 0x8001){//������֤
			  if(RES_DATA[31] == 0x7E){//ͬ��ʱ����
			  	unsigned char j=0;
					 for(j=0;j<6;j++){
						Timestamp[j] = RES_DATA[j+32];
				 }
					 
					 Timestamp[0] = Timestamp[0] ;
					 CheckTime = 1;
		 
		    }else  if(RES_DATA[31] == 0x01){//������֤���
					 
					 CheckAuth = 1;
		    }
			 
			 
		 }else if(dataCmdFlag ==0x8002){//ʵʱ��Ϣ�����ϱ�
			 
		 }else if(dataCmdFlag ==0x8003){//����
			 
		 }else if(dataCmdFlag ==0x8004){//�豸�ǳ�
			 
		 }else if(dataCmdFlag ==0x8005){//����
			 
		 }else if(dataCmdFlag ==0x8006){//Զ�̿���

			 if(RES_DATA[31] == 0x02){//�������ݲ�ѯ	�¶ȡ�ʪ�ȡ��ơ����ȣ��������Ϣ�嶨�塿
					unsigned char  light_status = PUMP ? 0x02 : 0x01;
					unsigned char buzzy_status = Buzzer ? 0x02 : 0x01;
					unsigned char xdata ds[37] = {0};
					unsigned char dslen =37;
			  	unsigned char j=0;
				
				  ds[0] = 0X23;//����ͷ
					ds[1] = 0X23;
					ds[2] = 0X10;//�����ʶ  �·�0x8006  ���ڵ��ϴ���0x1006
					ds[3] = 0X06;
					
					if(dataCmdAck == 0xFE){//Ӧ���ʶ
						ds[4] = 0x01;//�ɹ�
						
					}
			
				 for(j=0;j<17;j++){//CID��ֵ
						ds[j+5] = SRCCID[j];
				 }
				ds[22] = 0X01;//������
				ds[23] = 0X00;//������λ ��λ00
				ds[24] = 0X0B;//��λ0B һ��11λ

				ds[25] = Timestamp[0];//�� 0x14+2000 = 2020 
				ds[26] = Timestamp[1];//�� 
				ds[27] = Timestamp[2];//�� 
				ds[28] = Timestamp[3];//ʱ 
				ds[29] = Timestamp[4];//��
				ds[30] = Timestamp[5];//��
				
				ds[31] = 0X02;//������ѯ   ����


					ds[32] = DATA_Temphui[0]; //��������4���ֽڵ�����
					ds[33] = DATA_Temphui[1];
					ds[34] = light_status;
					ds[35] = buzzy_status;
					
			
					
				 ds[dslen-1] = CheckBCC(dslen, ds);//����У���  �����һλ
						SendAckData(dslen,ds);

				 
				 
			 }else if(RES_DATA[31] == 0x03){//��������	�ơ����ȣ��������Ϣ�嶨�塿
				 			 
					 unsigned char pump = RES_DATA[32];
					 unsigned char buzzy = RES_DATA[33];
			 
					 if( pump==0x02){
							Pump_Actions_Status(0);
						}else if( pump==0x01){
							Pump_Actions_Status(1);
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
						RES_DATA[4] = 0x01;//�ɹ�
					
					}
					if(dataCmdAck == 0xFE){//Ӧ���ʶ
						RES_DATA[4] = 0x01;//�ɹ�
						
					}
			
				 for(j=0;j<17;j++){//CID��ֵ
						RES_DATA[j+5] = SRCCID[j];
				 }
				RES_DATA[22] = 0X01;//������
				RES_DATA[23] = 0X00;//������λ ��λ00
				RES_DATA[24] = 0X09;//��λ09 һ��9λ    6λ��ʱ��+1λ�������ʶ + 2λ������

				RES_DATA[25] = Timestamp[0];//�� 0x14+2000 = 2020 
				RES_DATA[26] = Timestamp[1];//�� 
				RES_DATA[27] = Timestamp[2];//�� 
				RES_DATA[28] = Timestamp[3];//ʱ 
				RES_DATA[29] = Timestamp[4];//��
				RES_DATA[30] = Timestamp[5];//��
				
				RES_DATA[31] = 0X03;//��������  �ơ����ȣ��������Ϣ�嶨�塿
				
//				RES_DATA[32] = RES_DATA[32];// ����λ���øĶ�  
//				RES_DATA[33] = RES_DATA[33];
				
				
						RES_DATA[len-1] = CheckBCC(len, RES_DATA);//��һ֡���� 35���ֽ� len=35
						SendAckData(len,RES_DATA);

			 
			 }else if(RES_DATA[31] == 0x7F){//����
				 	IAP_CONTR = 0X20;
			 }
			 
			 
			 
		 }
		 
		//ˢ��һ��LED��Ļ
		LEDFunc(DATA_Temphui[0],DATA_Temphui[1]);
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



//��ʼ��LED�ͷ�����
void Device_Init() {

    LED = 0;
    Buzzer = 0;
		PUMP = 0;
}

//��ʼ����ɵε�����
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
    SCON = 0x50;                //8λ�ɱ䲨����
    PS = 1;
		
//  ACC = P_SW1;
//  ACC &= ~(S1_S0 | S1_S1);    //S1_S0=0 S1_S1=1
//  ACC |= S1_S1;               //(P1.6/RxD_3, P1.7/TxD_3)
//  P_SW1 = ACC;


    AUXR = 0x40;                //��ʱ��1Ϊ1Tģʽ
    TMOD = 0x00;                //��ʱ��1Ϊģʽ0(16λ�Զ�����)
    TL1 = (65536 - (FOSC/4/BAUD));   //���ò�������װֵ
    TH1 = (65536 - (FOSC/4/BAUD))>>8;
    TR1 = 1;                    //��ʱ��1��ʼ����
    ES = 1;                     //ʹ�ܴ����ж�
    EA = 1;

}

/*----------------------------
UART �жϷ������
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
        TI = 0;                 //���TIλ
        busy = 0;               //��æ��־
    }
}


void UART_T (unsigned char UART_data) { //���崮�ڷ������ݱ���
    SBUF = UART_data;	//�����յ����ݷ��ͻ�ȥ
    while(!TI);		//��鷢���жϱ�־λ
    TI = 0;			//����жϱ�־λΪ0��������㣩
}


void UART_TC (unsigned char *str) {
    while(*str != '\0') {
        UART_T(*str);
        *str++;
    }
    *str = 0;
}


//����  ����ESP8266�Ĵ������ݣ���У�����ݵ�������9λ

void UART_R()
{
		TL0 = 0x00;		//���ö�ʱ��ֵ
	TH0 = 0xDC;		//���ö�ʱ��ֵ
	TF0 = 0;		//���TF0��־
	TR0 = 1;		//��ʱ��0��ʼ��ʱ
	ET0 = 1;	//�����ж�
	
	

	 DATA_GET[CURRENT_LENGTH]=SBUF;
	 CURRENT_LENGTH++;
	
	

}



void SendAckData(U8 len, unsigned char *RES_DATA) {
	
		unsigned int i=0;
    for(i=0; i<len; i++)
    {
				 
				SBUF=RES_DATA[i];
				while(!TI);		//��鷢���жϱ�־λ
					TI = 0;	
		}
}

//���� ESP8266WiFiģ��
void ReConnectServer() {

    UART_TC("+++\0"); // �˳�͸��ģʽ
		 DELAY_MS( 1000);
    UART_TC("AT+RST\r\n\0");  // ��λ
		

		
}

//��ʼ��ESP8266WiFiģ�飬�����ӵ�������
void ConnectServer() {

    DELAY_MS( 1000);

    UART_TC("+++\0"); // �˳�͸��ģʽ
    DELAY_MS( 1000);
		
		UART_TC("AT+CWMODE=1\r\n\0"); // ��������STAģʽ
    DELAY_MS( 2500);
		
    UART_TC("AT+CIPMUX=0\r\n\0");  // ���õ�����ģʽ
    DELAY_MS(1000);

    UART_TC(netConfig);  // ��һ����������wifi����ʱ��ʱ��Ҫ��һЩ�������Ȳ������ص���Ϣ��10s
    DELAY_MS(15000);


    UART_TC("AT+CIPSTART=\"TCP\",\"47.104.19.111\",4001\r\n\0");	// ���ӵ�ָ��TCP������47.104.19.111
    DELAY_MS( 5000);

    UART_TC("AT+CIPMODE=1\r\n\0"); // ����͸��ģʽ
    DELAY_MS( 2000);

   UART_TC("AT+SAVETRANSLINK=1,\"47.104.19.111\",4001,\"TCP\"\r\n\0"); // ����TCP���ӵ�flash��ʵ���ϵ�͸��
   DELAY_MS(1000);

    UART_TC("AT+CIPSEND\r\n\0");	 // ����͸��ģʽ ׼��ģ������Խ��л�������
    DELAY_MS( 1000);
		
		

}

void Timer4Init(void)		
{
	//50 ����@11.0592MHz
	T4T3M &= 0xDF;		//��ʱ��ʱ��12Tģʽ
	T4L = 0x00;		//���ö�ʱ��ֵ
	T4H = 0x4C;		//���ö�ʱ��ֵ
	T4T3M |= 0x80;		//��ʱ��4��ʼ��ʱ
	
		IE2 |= 0x40;		//����ʱ��4�ж�
		EA=1; 	//���жϿ���
}


//10s�ж��Զ��ϱ���Ϣ
void Timer4_interrupt() interrupt 20    //��ʱ�ж����
{
	

	
	if(DisplayTime_Count>=20){  //20 * 50ms = 1s
		DisplayTime_Count = 0;
		
		if(CheckTime==1){  //�Ѿ�ͬ����������ʱ���
			
			Timestamp[5] =  Timestamp[5] + 1;//��
			
			if(Timestamp[5]>=60){
				Timestamp[5] = 0;
				Timestamp[4] = Timestamp[4] + 1;//��
				
				if(Timestamp[4]>=60){
					Timestamp[4] = 0;
					Timestamp[3] = Timestamp[3] + 1;//Сʱ
					
					if(Timestamp[3]>=24){
						Timestamp[3] = 0;
						Timestamp[2] = Timestamp[2] + 1;//��
						
						if( ((Timestamp[1] == 4 || Timestamp[1] == 6 || Timestamp[1] == 9 || Timestamp[1] == 11) && Timestamp[2]>=30) || ((Timestamp[1] == 1 || Timestamp[1] == 3 || Timestamp[1] == 5 || Timestamp[1] == 7 || Timestamp[1] == 8 || Timestamp[1] == 10 || Timestamp[1] == 12) && Timestamp[2]>=31) || ((((Timestamp[0] % 4 == 0 && Timestamp[0] % 100 != 0) || Timestamp[0] % 400 == 0) == 1  && Timestamp[2]>=29 ) ||  (((Timestamp[0] % 4 == 0 && Timestamp[0] % 100 != 0) || Timestamp[0] % 400 == 0) == 1  && Timestamp[2]>=28) ) ){
							Timestamp[2] = 1;
							Timestamp[1] = Timestamp[1] + 1;//��
							
							if(Timestamp[1]>=13){
								Timestamp[1] = 1;
								Timestamp[0] = Timestamp[0] + 1;//��
							 }
							
						 }
						
				 	}
					
			 	}	
				
		 	 }
			
	  	}
		
		
		
				if(CheckTime==0){//�豸ͬ��ʱ��
					
							unsigned char j=0;
							U8 xdata RES_DATA[65]= {0};
							unsigned char RES_LEN= 65;
							
							RES_DATA[0] = 0X23;//����ͷ
							RES_DATA[1] = 0X23;
							RES_DATA[2] = 0X10;//�����ʶ  �·�0x8006  ���ڵ��ϴ���0x1006
							RES_DATA[3] = 0X01;
							RES_DATA[4] = 0xFE;//Ӧ���ʶ
								
						 for(j=0;j<17;j++){//CID��ֵ
								RES_DATA[j+5] = SRCCID[j];
						 }
						 
						RES_DATA[22] = 0X01;//������
						RES_DATA[23] = 0X00;//������λ ��λ00
						RES_DATA[24] = 0X27;//��λ0x27 

						for(j=0;j<6;j++){//Timestamp
								RES_DATA[j+25] = Timestamp[j];
						 }
					
						
						RES_DATA[31] = 0X7E;//ͬ��ʱ��

						for(j=0;j<32;j++){//openid
								RES_DATA[j+32] = SRCOPENID[j];
						 }
					
					RES_DATA[RES_LEN-1] = CheckBCC(RES_LEN, RES_DATA);
												
					SendAckData(RES_LEN,RES_DATA);
					
				
				} else if( CheckAuth ==0 ){//�豸��½��֤
					
						unsigned char j=0;
							U8 xdata RES_DATA[65]= {0};
							unsigned char RES_LEN= 65;
							
							RES_DATA[0] = 0X23;//����ͷ
							RES_DATA[1] = 0X23;
							RES_DATA[2] = 0X10;//�����ʶ  �·�0x8006  ���ڵ��ϴ���0x1006
							RES_DATA[3] = 0X01;
							RES_DATA[4] = 0xFE;//Ӧ���ʶ
								
						 for(j=0;j<17;j++){//CID��ֵ
								RES_DATA[j+5] = SRCCID[j];
						 }
						 
						RES_DATA[22] = 0X01;//������
						RES_DATA[23] = 0X00;//������λ ��λ00
						RES_DATA[24] = 0X27;//��λ0x27 

						for(j=0;j<6;j++){//Timestamp
								RES_DATA[j+25] = Timestamp[j];
						 }
					
						
						RES_DATA[31] = 0X01;//��½��֤

						for(j=0;j<32;j++){//openid
								RES_DATA[j+32] = SRCOPENID[j];
						 }
					
					RES_DATA[RES_LEN-1] = CheckBCC(RES_LEN, RES_DATA);
												
					SendAckData(RES_LEN,RES_DATA);
					
					
				}else {
					
					
				}

		
		
		
	}else{
		 DisplayTime_Count++;//ÿ��һ�μ�50ms
	}
	

			
			
				if(Timer4_Count>=200){  //200 * 50ms = 10s
							unsigned char j=0;
							U8 xdata RES_DATA[37]= {0};
							unsigned char RES_LEN= 37;
							unsigned char  light_status = PUMP ? 0x02 : 0x01;
							unsigned char buzzy_status = Buzzer ? 0x02 : 0x01;
							Timer4_Count = 0;
	
							RES_DATA[0] = 0X23;//����ͷ
							RES_DATA[1] = 0X23;
							RES_DATA[2] = 0X10;//�����ʶ  �·�0x8006  ���ڵ��ϴ���0x1006
							RES_DATA[3] = 0X06;
							RES_DATA[4] = 0xFE;//Ӧ���ʶ
								
						 for(j=0;j<17;j++){//CID��ֵ
								RES_DATA[j+5] = SRCCID[j];
						 }
						 
						RES_DATA[22] = 0X01;//������
						RES_DATA[23] = 0X00;//������λ ��λ00
						RES_DATA[24] = 0X0B;//��λ0B һ��11λ

						RES_DATA[25] = Timestamp[0];//�� 0x14+2000 = 2020 
						RES_DATA[26] = Timestamp[1];//�� 
						RES_DATA[27] = Timestamp[2];//�� 
						RES_DATA[28] = Timestamp[3];//ʱ 
						RES_DATA[29] = Timestamp[4];//��
						RES_DATA[30] = Timestamp[5];//��
						
						RES_DATA[31] = 0X02;//���������ϱ�

					RES_DATA[32] = DATA_Temphui[0];
					RES_DATA[33] = 	DATA_Temphui[1];
					RES_DATA[34] = light_status;
					RES_DATA[35] = buzzy_status,
					RES_DATA[RES_LEN-1] = CheckBCC(RES_LEN, RES_DATA);
							
					Timeout_Count++;//ÿ��һ�μ�10s
					
					if(Timeout_Count < 3 && CheckTime==1 && CheckAuth ==1){ //û����֤�ɹ����˷�����Ҳ��ܾ�
						
						SendAckData(RES_LEN,RES_DATA);
				}else if(Timeout_Count >= 3){//30s�����ղ������������ص����ݣ��� ��������
						
						ReConnectServer();
						Timeout_Count = 0;
						
								//������֤
								CheckTime = 0;
								CheckAuth = 0;
							OLED_P6x8Str(0,7,"status: Closed    ");//Connected closed starting

					}
					
				}else{
					
						Timer4_Count++;
				}
				

		
}





void Timer0Init(void)		//10����@11.0592MHz
{
	AUXR &= 0x7F;		//��ʱ��ʱ��12Tģʽ
	TMOD &= 0xF0;		//���ö�ʱ��ģʽ
	TMOD |= 0x01;		//���ö�ʱ��ģʽ
	TL0 = 0x00;		//���ö�ʱ��ֵ
	TH0 = 0xB8;		//���ö�ʱ��ֵ
	TF0 = 0;		//���TF0��־
	TR0 = 1;		//��ʱ��0��ʼ��ʱ
	ET0 = 1;	//�����ж�
}



/********************* Timer0�жϺ���************************/
void timer0_int (void) interrupt 1
{
	TL0 = 0x00;		//���ö�ʱ��ֵ
	TH0 = 0xB8;		//���ö�ʱ��ֵ
	TF0 = 0;		//���TF0��־
	TR0 = 0;		//��ʱ��0��ʼ��ʱ
	ET0 = 0;	//�����ж�
	
	ResponseData(CURRENT_LENGTH,DATA_GET);		
	CURRENT_LENGTH = 0;
			
}



void Pump_Actions_Status(unsigned char status){

	if(status){
		PUMP = 0;
	}else{
		PUMP = 1;
	}

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