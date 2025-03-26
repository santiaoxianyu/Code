/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "headfile.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//char RxBuffer[256]; 
uint8_t aRxBuffer;			//接收中断缓冲
uint8_t Uart1_Rx_Cnt = 0;		//接收缓冲计数
// extern uint8_t RxBuffer[1];//串口接收缓冲
// extern uint8_t RxLine = 0;//指令长度
 extern uint8_t DataBuff[200];//指令内容
 extern int8_t flag;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Angle_out(void);
void Read(void);
void one_topic(void);
void two_topic(void);
void three_topic(void);
void four_topic(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
__IO uint8_t InitOK = 0;
//串口重定向
int fputc(int ch,FILE *f);
int fgetc(FILE *f);
//编码器、电机
uint32_t sys_tick; 
int Encoder1,Encoder2;
int16_t left,right,left_last,right_last;
int16_t L_Target_Position, L_Target_Speed;
//陀螺仪1
float yaww,yawl,yaw_init,yaw_out;
int Init_Angle_Flag=1;
float q0, q1, q2, q3;
float roll, pitch, yaw;
extern int8_t track_read[8];
//int8_t track_assignment[8];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	//oled初始化
	OLED_Init();
	OLED_Clear();
	OLED_ShowString(1,1,"oled_OK");
	HAL_Delay(3000);
	OLED_Clear();	
	//电机
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	//编码器
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);	
	//BNO085 IIC 初始化
	I2C_Init();  
	HAL_Delay(2000);//延时2秒
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);

	// uint8_t data;
	// HAL_UART_Receive_IT(&huart1, &data, 1);
	 printf("Init OK!!!\r\n");
    // 重置传感器
	softReset();

////	enableRotationVector(1);
////	enableGameRotationVector(100);
////	enableAccelerometer(100);
////	enableLinearAccelerometer(100);
////	enableGyro(100);
////	enableMagnetometer(100);
////	enableStepCounter(100);
////	enableStabilityClassifier(100);

	enableRotationVector(50);       // 增加旋转矢量传感器的采样率
	enableGameRotationVector(100);  // 保持游戏旋转矢量传感器的采样率
	enableAccelerometer(100);       // 保持加速度计的采样率
	enableLinearAccelerometer(100); // 保持线性加速度计的采样率
	enableGyro(100);                // 增加陀螺仪的采样率以提高精度
	enableMagnetometer(100);         // 降低磁力计的采样率以节省功耗
	enableStepCounter(50);          // 降低步数计数器的采样率
	enableStabilityClassifier(50);  // 降低稳定性分类器的采样率
	// 开始校准（可选）
	calibrateAll();//全部校准
	OLED_ShowString(1,1,"Init_OK");
	HAL_Delay(3000);
	OLED_Clear();		
	//初始化完成，开启中断
	HAL_TIM_Base_Start_IT(&htim4);	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	Load(0,0);
  int topic;
  while (1)
  {
    Track_follow();
    Read();
    Angle_out();
    OLED_ShowString(2,1,"yaw:");
	OLED_ShowSignedNum(2,5,yawl,3);
    topic=4;
    switch (topic)
    {
      case 1:
      {
        /* code */
        int8_t pattern;
        if(pattern==0)Load(500,500);	
        for(int8_t i=0;i<=7;i++)
        {
          if(track_read[i]==0)
          {
            pattern=1;
            Load(0,0);
          }		
        }     
        break;
      }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
  if(Uart1_Rx_Cnt >= 255)  //溢出判断
  {
      Uart1_Rx_Cnt = 0;
      memset(DataBuff, 0x00, sizeof(DataBuff));  // 清空接收缓冲区
      HAL_UART_Transmit(&huart1, (uint8_t *)"数据溢出", 10, 0xFFFF);  // 发送溢出提示
  }
  else
  {
      DataBuff[Uart1_Rx_Cnt++] = aRxBuffer;   // 接收数据转存到 DataBuff
  
      // 判断结束位（0x0D 0x0A，即 \r\n）
      if((DataBuff[Uart1_Rx_Cnt-1] == 0x0A) && (DataBuff[Uart1_Rx_Cnt-2] == 0x0D)) 
      {
          // 将收到的数据回传
          HAL_UART_Transmit(&huart1, (uint8_t *)&DataBuff, Uart1_Rx_Cnt, 0xFFFF);
          
          // 等待发送完成（可选，根据需求决定是否阻塞）
          while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
  
          // 重置计数器和缓冲区
          Uart1_Rx_Cnt = 0;
          memset(DataBuff, 0x00, sizeof(DataBuff));
      }
  }
	// if(Uart1_Rx_Cnt >= 255)  //溢出判断
	// {
	// 	Uart1_Rx_Cnt = 0;
	// 	memset(RxBuffer,0x00,sizeof(RxBuffer));
	// 	HAL_UART_Transmit(&huart1, (uint8_t *)"数据溢出", 10,0xFFFF); 	
        
	// }
	// else
	// {
	// 	RxBuffer[Uart1_Rx_Cnt++] = aRxBuffer;   //接收数据转存
	
	// 	if((RxBuffer[Uart1_Rx_Cnt-1] == 0x0A)&&(RxBuffer[Uart1_Rx_Cnt-2] == 0x0D)) //判断结束位
	// 	{
	// 		HAL_UART_Transmit(&huart1, (uint8_t *)&RxBuffer, Uart1_Rx_Cnt,0xFFFF); //将收到的信息发送出去
  //           while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);//检测UART发送结束
	// 		Uart1_Rx_Cnt = 0;
	// 		memset(RxBuffer,0x00,sizeof(RxBuffer)); //清空数组
	// 	}

	// }
	
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);   //再开启接收中断
}
/* USER CODE END 4 */
// int fputc(int ch,FILE *f) 
// {
// //采用轮询方式发送1字节数据，超时时间设置为无限等待
// HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,HAL_MAX_DELAY);
// return ch;
// }

// int fgetc(FILE *f)
// {
// uint8_t ch;
// // 采用轮询方式接收 1字节数据，超时时间设置为无限等待
// HAL_UART_Receive( &huart1,(uint8_t*)&ch,1, HAL_MAX_DELAY );
// return ch;
// }

void Read(void)
{
	if(uwTick-sys_tick<10)           
		return;
	sys_tick=uwTick;                 
	Encoder1=Read_Speed(&htim2);
	Encoder2=-Read_Speed(&htim3);    
	OLED_ShowString(1,1,"T:");
	OLED_ShowString(1,8,"F:");
	OLED_ShowSignedNum(1,3,Encoder1,3);
	OLED_ShowSignedNum(1,10,Encoder2,3);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if ( dataAvailable() )
  {
      q0 = getQuatReal();
      q1 = getQuatI();
      q2 = getQuatJ();
      q3 = getQuatK();
		 
      roll = atan2( 2 * ( q0 * q1 + q2 * q3 ) ,  1- 2 * ( q1 * q1 + q2 * q2 ) ) * 57.3;
      pitch = asin( 2 * ( q0 * q2 - q3 * q1 ) ) * 57.3;
      yaw = atan2( 2 * ( q0 * q3 + q1 * q2 ) ,  1 - 2 * ( q2 * q2 + q3 * q3 ) ) * 57.3;
      
      printf("%f,%f,%f\n", roll, pitch, yaw);
  }  		
  if(flag==0)steering_ring();							
  if(flag==1)Track_ring();
}

void Angle_out(void)
{
		if(Init_Angle_Flag==1&&yaw!=0)
		{
				yaw_init=yaw;
			  Init_Angle_Flag=0;			
	    }
		 yaww=yaw-yaw_init;
		 
		 if(yaww>180)
		 {
			yaww=yaww-360;
		 } 
		 else if(yaww<-180)
		 {
			yaww=yaww+360;
		 }	 
		 else
		 {
			 yaww=yaw-yaw_init;			 
		 }
		  yawl=yaww;
//		 OLED_ShowSignedNum(3,1,yawl,3);
}

void one_topic(void)
{
  int8_t pattern;
	if(pattern==0)Load(500,500);	
	for(int8_t i=0;i<=7;i++)
	{
		if(track_read[i]==0)
		{
			pattern=1;
      Load(0,0);
		}		
	}
//  steering_ring(0,0);//转向环
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
