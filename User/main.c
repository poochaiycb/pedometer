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


/* MPU6050数据 */
short Acel[3];
short Gyro[3];
float Temp;


extern uint8_t hour = 20;
extern uint8_t min  = 10;
extern uint8_t sec  = 0;
int time1,time2,time3;


//校0
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
    /* 计算x, y, z 轴倾角，返回弧度值*/
    data->X_Angle = acos((Acel[0]-X_ACCEL_OFFSET) / 16384.0);
    data->Y_Angle = acos((Acel[1]-Y_ACCEL_OFFSET) / 16384.0);
    data->Z_Angle = acos((Acel[2]-Z_ACCEL_OFFSET) / 16384.0);

    /* 弧度值转换为角度值 */
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
//读取的ID存储位置 
__IO uint32_t DeviceID = 0;
__IO uint32_t FlashID = 0;


/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{
	MPU6050_Angle data;
	unsigned char string[10] = {0};
	unsigned char string1[20] = {0};
	int step = 0;
	uint8_t cal_flag = 0; //flash标志位
	int int_bufffer[10] = {0};
	
	
	SPI_FLASH_Init();
	
	/* LED 端口初始化 */
  LED_GPIO_Config();

  /*初始化USART1*/
  Debug_USART_Config(); 
	
//	EEPROM_GPIO_Config();
  BASIC_TIMx_Config();
	
	
  /* 配置SysTick定时器和中断 */
  SysTick_Init(); //配置 SysTick 为 1ms 中断一次，在中断里读取传感器数据
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //启动定时器
	
	I2C_Configuration();//配置CPU的硬件I2C
	//OLED_Init();    //初始化OLED
	
	//初始化 I2C
	I2cMaster_Init(); 
//  EEPROM_I2C_ModeConfig();
	OLED_Init();    //初始化OLED


  //MPU6050初始化
	MPU6050_Init();
	//检测MPU6050
	
	//点亮OLED
	OLED_Fill(0xFF);//全屏点亮
		Delay_s(1);		// 1s
		
		OLED_Fill(0x00);//全屏灭
		Delay_s(1);		// 1s
		
	//OLED_OFF();//测试OLED休眠
	
	/* 获取 Flash Device ID */
	DeviceID = SPI_FLASH_ReadDeviceID();
	
	delay_ms(200);
	
	/* 获取 SPI Flash ID */
	FlashID = SPI_FLASH_ReadID();
	
	
	if( MPU6050ReadID() == 0 )
  {
		printf("\r\nError in MPU6060！\r\n");
		LED_RED; //故障
    
		while(1);	//检测不到MPU6050 会红灯亮然后卡死
	}
	
	
	SPI_FLASH_BufferRead(&cal_flag, SPI_FLASH_PageSize*0, 1);
    
		if( cal_flag == 0xCD )	/*若标志等于0xcd，表示之前已有写入数据*/
    {
			  //printf("\r\n检测到数据标志\r\n");
				
				/*读取整数数据*/
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
			/*擦除扇区*/
      SPI_FLASH_SectorErase(0);
			/*写入标志到第0页*/
      SPI_FLASH_BufferWrite(&cal_flag, SPI_FLASH_PageSize*0, 1); 
      
		  int_bufffer[0] = hour;
			int_bufffer[1] = min;
			int_bufffer[2] = sec;
		  SPI_FLASH_BufferWrite((void*)int_bufffer, SPI_FLASH_PageSize*1, sizeof(int_bufffer));
		
    if( task_readdata_finish ) //task_readdata_finish = 1 表示读取MPU6050数据完成
    {
      
			MPU6050_Get_Angle(&data);  // 计算三轴倾角   

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
      OLED_ShowStr(0,0,(unsigned char*)"Welcome!",2);				//测试6*8字符
		//OLED_ShowStr(0,3,(unsigned char*)"The step is",2);				//测试8*16字符	
		  sprintf((char *)string,"step: %d",step);
		  OLED_ShowStr(0,3,string,2);
			sprintf((char *)string1,"time %d:%d:%d",int_bufffer[0],int_bufffer[1],int_bufffer[2]);
			OLED_ShowStr(0,5,string1,2);
		  //delay_s(1);	
			//OLED_CLS();//清屏
		  //OLED_OFF();//OLED休眠
      //}
      task_readdata_finish = 0; // 清零标志位
    }
  }

}







