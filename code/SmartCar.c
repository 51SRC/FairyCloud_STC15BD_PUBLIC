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

//��ʾ����Keil������������ѡ��Intel��8058оƬ�ͺŽ��б���
//�����ر�˵��,����Ƶ��һ��Ϊ11.0592MHz


#include "STC15W4K58S4.h"

#include "intrins.h"
#include <string.h>  // �ַ�������ͷ�ļ�

sbit LED = P3 ^ 2;  // ��ӦӲ������
sbit LOUND = P5 ^ 4;  // ��ӦӲ������

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



#define FOSC 11059200L          //ϵͳƵ��
#define BAUD 115200             //���ڲ�����

#define S1_S0 0x40              //P_SW1.6
#define S1_S1 0x80              //P_SW1.7


void SendString(char *s);
void SendDatas(char *s);

void SendData(char *s);
void UART_TC (unsigned char *str);
void UART_T (unsigned char UART_data); //���崮�ڷ������ݱ���
void UART_R();//��������
void DELAY_MS(unsigned int timeout);		//@11.0592MHz   1ms
void ConnectServer();//���ӷ�����
void USART_Init();
void Device_Init();
void ResponseData(unsigned char *RES_DATA);
unsigned char CheckData(unsigned char *CHECK_DATA);
void sendAckData(unsigned char *RES_DATA);
void ConnectSuccess();
void Heat();
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

    };
}


void Device_Init() {

    LED = 0;
    LOUND = 0;
}

void ConnectSuccess(){

	 LOUND = 1;
	 DELAY_MS(200);
		LOUND = 0;
	 DELAY_MS(200);
	  LOUND = 1;
	 DELAY_MS(200);
	  LOUND = 0;

}


void Heat(){

	 LED = 1;
	 DELAY_MS(200);
		LED = 0;
	 DELAY_MS(200);
	  LED = 1;
	 DELAY_MS(200);
	  LED = 0;

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


/*----------------------------
���ʹ�������
----------------------------*/

void  SendData(char *s)
{

    unsigned int i=0;


    for(i=0; i<DATA_LENGTH; i++)
    {
				 
        SBUF=s[i];
     while(!TI);		//��鷢���жϱ�־λ
    TI = 0;	
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


//����  ���յ�������

void UART_R()
{
		if((CURRENT_LENGTH==0)&&(SBUF==0x7E))// �жϵ�һ���ǲ���0x7e  ��ȷ���ǲ���β��7e
	   {go=1;}

	if((go==1)&&(CURRENT_LENGTH==1)&&(SBUF==0X7E))//�ڶ���7e
	   {CURRENT_LENGTH=0;} 

	if(go==1)  //10���ַ�����������
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

   
	
//		if(CURRENT_LENGTH==DATA_LENGTH )
//    {
//				if(DATA_GET[0] == 0x7E && DATA_GET[DATA_LENGTH-1] == 0x7E ){
//						CURRENT_LENGTH=0;
//						ResponseData(DATA_GET);
//				}else {
//				
//				}
//       
//    }else if(	CURRENT_LENGTH==2 && DATA_GET[0]==0x7E && DATA_GET[1]==0x7E){
//			CURRENT_LENGTH = 1;
//		}

		

}



///У������׼ȷ�� ��CRCУ��
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


void ResponseData(unsigned char *RES_DATA) {

    if((CheckData(RES_DATA) == RES_DATA[DATA_LENGTH-2]) && RES_DATA[1]== SRCDeviceID &&  RES_DATA[2]== SRCAID && RES_DATA[4]== 0x01 ) {

        if(RES_DATA[3]==0x00 ) {
					RES_DATA[3]=0x04;//����λ���� 4������ʪ��ָ��
					RES_DATA[5]= 0x11;//����λ����
					RES_DATA[6]= 0x13;//����ת��  ����λ����λ

            sendAckData(RES_DATA);
						Heat();
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


void ConnectServer() {

    DELAY_MS( 1000);

    UART_TC("+++\0"); // �˳�͸��ģʽ
    DELAY_MS( 1000);

//    UART_TC("AT+RST\r\n\0");  // ��λ
//    DELAY_MS(2000);
		
		UART_TC("AT+CWMODE=1\r\n\0"); // ��������STAģʽ
    DELAY_MS( 2500);
		
    UART_TC("AT+CIPMUX=0\r\n\0");  // ���õ�����ģʽ
    DELAY_MS(1000);

    UART_TC("AT+CWJAP=\"Gunter\",\"{qwerty123}\"\r\n\0");  // ��һ����������wifi����ʱ��ʱ��Ҫ��һЩ�������Ȳ������ص���Ϣ��10s
    DELAY_MS(10000);


    UART_TC("AT+CIPSTART=\"TCP\",\"47.104.19.111\",4001\r\n\0");	// ���ӵ�ָ��TCP������
    DELAY_MS( 5000);

    UART_TC("AT+CIPMODE=1\r\n\0"); // ����͸��ģʽ
    DELAY_MS( 2000);

  // UART_TC("AT+SAVETRANSLINK=1,\"192.168.0.11\",4001,\"TCP\"\r\n\0"); // ����TCP���ӵ�flash��ʵ���ϵ�͸��
  // DELAY_MS(1000);

    UART_TC("AT+CIPSEND\r\n\0");	 // ����͸��ģʽ ׼��ģ������Խ��л�������
    DELAY_MS( 1000);

    CURRENT_LENGTH=0;
		
		

}



void Timer4Init(void)		//5����@11.0592MHz
{
	T4T3M |= 0x20;		//��ʱ��ʱ��1Tģʽ
	T4L = 0x00;		//���ö�ʱ��ֵ
	T4H = 0x28;		//���ö�ʱ��ֵ
	T4T3M |= 0x80;		//��ʱ��4��ʼ��ʱ
		IE2 |= 0x40;		//����ʱ��4�ж�
		EA=1; 	//���жϿ���
}


//�жϷ������
void Timer4_interrupt() interrupt 20           //�ж����

{


		if(Timer4_Count>=2500){
			Timer4_Count = 1;
			

			RES_DATA[3]=0x04;//����λ���� 4������ʪ��ָ��
					RES_DATA[5]= 0x66;//����λ����
					RES_DATA[6]= 0x67;//����ת��  ����λ����λ
			
			sendAckData(RES_DATA);
		}else{
			
		Timer4_Count++;
		}
}
