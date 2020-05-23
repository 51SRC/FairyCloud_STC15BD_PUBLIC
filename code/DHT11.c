#include "DHT11.h"
#include "mcuinit.h"

sbit DQ=P4^5;

void DHT11_Rst(void);
unsigned char DHT11_Check(void);
unsigned char DHT11_Read_Bit(void);
unsigned char DHT11_Read_Byte(void);
unsigned char DHT11_Read_Data(unsigned char *temp,unsigned char *humi);
 unsigned char DATA_Temphui[3]={0x00,0x00,0x00};

void DELAY_CMS(unsigned char ms){
    unsigned int i;
		do{
	      i = 11059200UL / 13000;
		  while(--i)	;
     }while(--ms);
}


void Delay1us()		//@11.0592MHz
{
	_nop_();
	_nop_();
	_nop_();
}
void Delay30us()		//@11.0592MHz
{
	unsigned char i;
	_nop_();
	_nop_();
	i = 80;
	while (--i);
}

void Delay50us()		//@11.0592MHz
{
	unsigned char i, j;

	_nop_();
	i = 1;
	j = 134;
	do
	{
		while (--j);
	} while (--i);
}

/*
unsigned char DHT11()
{
	unsigned char humi=0x00,temp=0x00;

	if(DHT11_Read_Data(&temp,&humi)==0)
	{
		return 0;
	}
	else
		return 1;
}
*/


//��λDHT11
void DHT11_Rst(void)	   
{                 
	DQ=0;           	//����DQ
    DELAY_CMS(20);    	//��������18ms
    DQ=1; 	            //DQ=1 
	Delay30us();     	//��������20~40us
}

//�ȴ�DHT11�Ļ�Ӧ
//����1:δ��⵽DHT11�Ĵ���
//����0:����
unsigned char DHT11_Check(void) 	   
{   
	unsigned char retry=0; 
    while(DQ&&retry<100)//DHT11������40~80us
	{
		retry++;
		Delay1us();
	};	 
	if(retry>=100)return 1;
	else retry=0;
    while (!DQ&&retry<100)//DHT11���ͺ���ٴ�����40~50us
	{
		retry++;
		Delay1us();
	};
	if(retry>=100)return 1;	    
	return 0;
}

//��DHT11��ȡһ��λ
//����ֵ��1/0
unsigned char DHT11_Read_Bit(void) 			 
{
 	unsigned char retry=0;
	while(DQ&&retry<100)//�ȴ���Ϊ�͵�ƽ
	{
		retry++;
		Delay1us();
	}
	retry=0;
	while(!DQ&&retry<100)//�ȴ���ߵ�ƽ
	{
		retry++;
		Delay1us();
	}
	Delay50us();//�ȴ�50us
	if(DQ)return 1;
	else return 0;		   
}

//��DHT11��ȡһ���ֽ�
//����ֵ������������
unsigned char DHT11_Read_Byte(void)    
{        
    unsigned char i,dat;
    dat=0;
	for(i=0;i<8;i++) 
	{
   		dat<<=1; 
	    dat|=DHT11_Read_Bit();
    }						    
    return dat;
}

//��DHT11��ȡһ������
//temp:�¶�ֵ(��Χ:0~50��)
//humi:ʪ��ֵ(��Χ:20%~90%)
//����ֵ��0,����;1,��ȡʧ��
unsigned char DHT11_Read_Data(unsigned char *temp,unsigned char *humi)    
{        
 	unsigned char buf[5];
	unsigned char i;
	DHT11_Rst();
	if(DHT11_Check()==0)
	{
		for(i=0;i<5;i++)//��ȡ40λ����
		{
			buf[i]=DHT11_Read_Byte();
		}
		if((buf[0]+buf[1]+buf[2]+buf[3])==buf[4])
		{
			*humi=buf[0];	
			*temp=buf[2];
		}
	}
	else return 1;
	return 0;	    
}


