#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "./usart/bsp_debug_usart.h"
#include "./led/bsp_led.h"
#include "./i2c/i2c.h"
#include "./mpu6050/mpu6050.h"
#include <math.h>
#include "./oled/OLED_I2C.h"
#include "./systick/bsp_SysTick.h"
#include "./basic-tim/bsp_basic_tim.h"
#include "./flash/bsp_spi_flash.h"


/* MPU6050���� */
short Acel[3];
short Gyro[3];
float Temp;


extern uint8_t hour = 20;
extern uint8_t min  = 10;
extern uint8_t sec  = 0;
int time1,time2,time3;


//У0
#define X_ACCEL_OFFSET -15 
#define Y_ACCEL_OFFSET 162 
#define Z_ACCEL_OFFSET 335
#define X_GYRO_OFFSET -36 
#define Y_GYRO_OFFSET 22 
#define Z_GYRO_OFFSET 14

typedef struct Angle
{
    double X_Angle;
    double Y_Angle;
    double Z_Angle;
    
} MPU6050_Angle;


void MPU6050_Get_Angle(MPU6050_Angle *data)
{   
    /* ����x, y, z ����ǣ����ػ���ֵ*/
    data->X_Angle = acos((Acel[0]-X_ACCEL_OFFSET) / 16384.0);
    data->Y_Angle = acos((Acel[1]-Y_ACCEL_OFFSET) / 16384.0);
    data->Z_Angle = acos((Acel[2]-Z_ACCEL_OFFSET) / 16384.0);

    /* ����ֵת��Ϊ�Ƕ�ֵ */
    data->X_Angle = data->X_Angle * 57.29577;
    data->Y_Angle = data->Y_Angle * 57.29577;
    data->Z_Angle = data->Z_Angle * 57.29577;
} 

void delay_ms(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=12000;  
      while(i--) ;    
   }
}

void delay_s(u16 time)
{
	u16 i=0; 
	for(;i<10;i++)
	delay_ms(1000);
}
//flash
//��ȡ��ID�洢λ�� 
__IO uint32_t DeviceID = 0;
__IO uint32_t FlashID = 0;


/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{
	MPU6050_Angle data;
	unsigned char string[10] = {0};
	unsigned char string1[20] = {0};
	int step = 0;
	uint8_t cal_flag = 0; //flash��־λ
	int int_bufffer[10] = {0};
	
	
	SPI_FLASH_Init();
	
	/* LED �˿ڳ�ʼ�� */
  LED_GPIO_Config();

  /*��ʼ��USART1*/
  Debug_USART_Config(); 
	
//	EEPROM_GPIO_Config();
  BASIC_TIMx_Config();
	
	
  /* ����SysTick��ʱ�����ж� */
  SysTick_Init(); //���� SysTick Ϊ 1ms �ж�һ�Σ����ж����ȡ����������
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //������ʱ��
	
	I2C_Configuration();//����CPU��Ӳ��I2C
	//OLED_Init();    //��ʼ��OLED
	
	//��ʼ�� I2C
	I2cMaster_Init(); 
//  EEPROM_I2C_ModeConfig();
	OLED_Init();    //��ʼ��OLED


  //MPU6050��ʼ��
	MPU6050_Init();
	//���MPU6050
	
	//����OLED
	OLED_Fill(0xFF);//ȫ������
		Delay_s(1);		// 1s
		
		OLED_Fill(0x00);//ȫ����
		Delay_s(1);		// 1s
		
	//OLED_OFF();//����OLED����
	
	/* ��ȡ Flash Device ID */
	DeviceID = SPI_FLASH_ReadDeviceID();
	
	delay_ms(200);
	
	/* ��ȡ SPI Flash ID */
	FlashID = SPI_FLASH_ReadID();
	
	
	if( MPU6050ReadID() == 0 )
  {
		printf("\r\nError in MPU6060��\r\n");
		LED_RED; //����
    
		while(1);	//��ⲻ��MPU6050 ������Ȼ����
	}
	
	
	SPI_FLASH_BufferRead(&cal_flag, SPI_FLASH_PageSize*0, 1);
    
		if( cal_flag == 0xCD )	/*����־����0xcd����ʾ֮ǰ����д������*/
    {
			  //printf("\r\n��⵽���ݱ�־\r\n");
				
				/*��ȡ��������*/
        SPI_FLASH_BufferRead((void*)int_bufffer, SPI_FLASH_PageSize*1, sizeof(int_bufffer));
			hour = int_bufffer[0];
			min = int_bufffer[1];
			sec = int_bufffer[2];
    } 
		
		else
    {
			hour = 20;
			min = 10;
			sec = 00;
			cal_flag =0xCD;
			
		}

  while(1)
  {
		  cal_flag =0xCD;
			/*��������*/
      SPI_FLASH_SectorErase(0);
			/*д���־����0ҳ*/
      SPI_FLASH_BufferWrite(&cal_flag, SPI_FLASH_PageSize*0, 1); 
      
		  int_bufffer[0] = hour;
			int_bufffer[1] = min;
			int_bufffer[2] = sec;
		  SPI_FLASH_BufferWrite((void*)int_bufffer, SPI_FLASH_PageSize*1, sizeof(int_bufffer));
		
    if( task_readdata_finish ) //task_readdata_finish = 1 ��ʾ��ȡMPU6050�������
    {
      
			MPU6050_Get_Angle(&data);  // �����������   

			if(data.Z_Angle > 40)
			{
				step++;
			}
			
			 // printf("X_Angle = %lf", data.X_Angle);
       // printf("  Y_Angle = %lf", data.Y_Angle);
			
//        printf("  Z_Angle = %lf", data.Z_Angle);
//				printf(" step = %d",step);
//        printf("\r\n");
			
			//printf("time =%d:%d:%d\n",time1,time2,time3);
			
     // printf("accel_x: %d accel_y: %d accel_z: %d",Acel[0]-X_ACCEL_OFFSET,Acel[1]-Y_ACCEL_OFFSET,Acel[2]-Z_ACCEL_OFFSET);
      
     // printf("  Gyro_x: %d Gyro_y: %d Gyro_z: %d\r\n",Gyro[0]-X_GYRO_OFFSET,Gyro[1]-Y_GYRO_OFFSET,Gyro[2]-Z_GYRO_OFFSET);
      //if(data.Z_Angle > 20)
			//{
			//OLED_ON();
      OLED_ShowStr(0,0,(unsigned char*)"Welcome!",2);				//����6*8�ַ�
		//OLED_ShowStr(0,3,(unsigned char*)"The step is",2);				//����8*16�ַ�	
		  sprintf((char *)string,"step: %d",step);
		  OLED_ShowStr(0,3,string,2);
			sprintf((char *)string1,"time %d:%d:%d",int_bufffer[0],int_bufffer[1],int_bufffer[2]);
			OLED_ShowStr(0,5,string1,2);
		  //delay_s(1);	
			//OLED_CLS();//����
		  //OLED_OFF();//OLED����
      //}
      task_readdata_finish = 0; // �����־λ
    }
  }

}







