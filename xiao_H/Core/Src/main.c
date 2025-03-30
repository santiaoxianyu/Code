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
//#include "track.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "headfile.h"
#include "track.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//char RxBuffer[256]; 
// uint8_t aRxBuffer;			//接收中断缓冲
// uint8_t Uart1_Rx_Cnt = 0;		//接收缓冲计数
uint8_t RxBuffer[1];//串口接收缓冲
uint8_t RxLine = 0;//指令长度
extern uint8_t DataBuff[200];//指令内容
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
void steering_ring(void);//转向�?
void Track_ring(void);//寻迹�?
void one_topic(void);
void two_topic(void);
void three_topic(void);
void four_topic(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//板子上的电机2的PID控制�?
PID_Controller left_pid = {
    .p=235.0f,
    .i=13.0f
}; 
//板子上的电机1的PID控制�?
PID_Controller right_pid = {
    .p=235.0f,
    .i=13.0f
}; 
//直走pid控制�?
PID_Controller yaw_pid={
    .p=1.6f,
    .d=1.2f
  };
//寻迹pid控制�?  
PID_Controller track_pid_assignment={
//	.p=2.0f
    .p=3.0f,
    .d=5.5f	
};
__IO uint8_t InitOK = 0;
//串口重定�?
int fputc(int ch,FILE *f);
int fgetc(FILE *f);
//转向环变�?
int16_t flag;
int16_t straight_right;
int16_t straight_left;
int16_t pid_turn;
extern unsigned int  rawGyroZ;

//寻迹环变�?
int16_t track_turn,track_left,track_right;
//编码器�?�电�?
uint32_t sys_tick; 
int Encoder1,Encoder2;
int Encoder1_last,Encoder2_last;
int16_t left,right,left_last,right_last;
int16_t L_Target_Position, L_Target_Speed;
extern int8_t track_sum;
//�?螺仪1
float yaww,yawl,yaw_init,yaw_out;
int Init_Angle_Flag=1;
float q0, q1, q2, q3;
float roll, pitch, yaw;
extern int8_t track_read[8];

//按键读高低电�?
struct keys{
  int8_t keys_read;
	int8_t keys_progres;
	int8_t keys_flag;
}key_pd[4]={0,0,0};


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
	//oled初始�?
	OLED_Init();
//	//电机
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
//	//编码�?
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);	
	//BNO085 IIC 初始�?
  HAL_UART_Receive_IT(&huart1, RxBuffer, 1);
//	 printf("Init OK!!!\r\n");
	// �?始校准（可�?�）
    // 重置传感�?
	softReset();
	enableRotationVector(2);       // 增加旋转矢量传感器的采样�?
//	enableGameRotationVector(50);  // 保持游戏旋转矢量传感器的采样�?
//	enableAccelerometer(50);       // 保持加�?�度计的采样�?
//	enableLinearAccelerometer(50); // 保持线�?�加速度计的采样�?
	enableGyro(100);                // 增加�?螺仪的采样率以提高精�?
//	enableMagnetometer(50);         // 降低磁力计的采样率以节省功�??
//	enableStepCounter(50);          // 降低步数计数器的采样�?
//	enableStabilityClassifier(50);  // 降低稳定性分类器的采样率
	calibrateAll();//全部校准
		
	//初始化完成，�?启中�?
	HAL_TIM_Base_Start_IT(&htim4);	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	Load(0,0);
  int topic;
//  L_Target_Speed=30;
  flag=1;
//	OLED_ShowString(1,1,"T:");
//	OLED_ShowString(1,8,"F:");

//  L_Target_Position=30;
  while (1)
  {
//    SendDataToVOFA(L_Target_Speed,Encoder1,Encoder2);
		
    Track_follow(); 
//    Read();  
    OLED_ShowString(2,1,"yaw:");
	  OLED_ShowSignedNum(2,5,yawl,3);
//	  OLED_ShowSignedNum(3,0,key_pd[0].keys_flag,3);
//	  OLED_ShowSignedNum(3,5,key_pd[1].keys_flag,3);
//	  OLED_ShowSignedNum(4,0,key_pd[2].keys_flag,3);
//	  OLED_ShowSignedNum(4,5,key_pd[3].keys_flag,3);
//	  OLED_ShowSignedNum(3,1,16-track_turn,4);
//	  OLED_ShowSignedNum(3,6,16+track_turn,4);
//		if (key_pd3.keys_falg == 1) {
//			L_Target_Speed = (L_Target_Speed == 0) ? 30 : 0;			
//			key_pd3.keys_falg =0; 
//		}
//	  OLED_ShowSignedNum(4,0,track_right,5);
//	  OLED_ShowSignedNum(4,7,track_left,5);
		

		//    Load(2000,2000);
    // SendDataToVOFA(L_Target_Speed,left,right);
    // topic=4;
//		 switch (key_pd[i])
//		 {
//			 case 1:
//			 {
//				 /* code */
//				 int8_t pattern;
//				 if(pattern==0)Load(500,500);	
//				 for(int8_t i=0;i<=7;i++)
//				 {
//					 if(track_read[i]==0)
//					 {
//						 pattern=1;
//						 Load(0,0);
//					 }		
//				 }     
//				 break;
//			 }
//		 }

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
//串口 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
//  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
  if(huart->Instance == USART1)//如果是串�?2
  {
      RxLine++;                      //每接收到�?个数据，进入回调数据长度�?1
      DataBuff[RxLine-1]=RxBuffer[0];  //把每次接收到的数据保存到缓存数组
      if(RxBuffer[0]==0x21)            //接收结束标志位，这个数据可以自定义，根据实际�?求，这里只做示例使用，不�?定是0x21
      {
          printf("RXLen=%d\r\n",RxLine);
          for(int i=0;i<RxLine;i++)
              printf("UART DataBuff[%d] = %c\r\n",i,DataBuff[i]);
          USART_PID_Adjust(1);//数据解析和参数赋值函�?
          memset(DataBuff,0,sizeof(DataBuff));  //清空缓存数组
          RxLine=0;  //清空接收长度
      }
      RxBuffer[0]=0;
      HAL_UART_Receive_IT(&huart1, RxBuffer, 1); //每接收一个数据，就打�?�?次串口中断接收，否则只会接收�?个数据就停止接收
  }
}
//看编码器数据
void Read(void)
{
//	if(uwTick-sys_tick<10)           
//		return;
//	sys_tick=uwTick;                 
	OLED_ShowSignedNum(1,3,Encoder1,3);
	OLED_ShowSignedNum(1,10,Encoder2,3);
}
//中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	Encoder1=-Read_Speed(&htim2);
	Encoder2=Read_Speed(&htim3);  
	
	Encoder2=0.6*Encoder2+0.4*Encoder2_last;
	Encoder1=0.6*Encoder1+0.4*Encoder1_last;
	
  if(flag==0)steering_ring();							
  if(flag==1)Track_ring();
	Encoder2_last=Encoder2;
	Encoder1_last=Encoder1;

	if ( dataAvailable() )
  {
      q0 = getQuatReal();
      q1 = getQuatI();
      q2 = getQuatJ();
      q3 = getQuatK();
      yaw = atan2( 2 * ( q0 * q3 + q1 * q2 ) ,  1 - 2 * ( q2 * q2 + q3 * q3 ) ) * 57.3;
  }
   Angle_out();
	
// 读取GPIOD端口上PIN_0到PIN_3的状态，并将读取的值分别存储在key_pd数组的keys_read成员中
key_pd[0].keys_read = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0); // 读取PIN_0的状态
key_pd[1].keys_read = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1); // 读取PIN_1的状态
key_pd[2].keys_read = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2); // 读取PIN_2的状态
key_pd[3].keys_read = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3); // 读取PIN_3的状态

// 遍历key_pd数组中的每个按键，检查其状态并更新相应的进度和标志位
for (int8_t i = 0; i < 4; i++) // 循环变量i从0开始，直到小于4（即遍历key_pd数组的前4个元素）
{
    switch (key_pd[i].keys_progres) // 根据当前按键的进度状态进行不同的处理
    {
        case 0: // 当按键进度为0时
        {
            if (key_pd[i].keys_read == 0) // 如果按键被按下（假设低电平表示按下）
            {
                key_pd[i].keys_progres = 1; // 将按键进度更新为1
            }
        } break;
        case 1: // 当按键进度为1时
        {
            if (key_pd[i].keys_read == 0) // 如果按键仍然被按下
            {
                key_pd[i].keys_progres = 2; // 将按键进度更新为2
            }
            else
            {
                key_pd[i].keys_progres = 0; // 如果按键被释放，将进度重置为0
            }
        } break;
        case 2: // 当按键进度为2时
        {
            if (key_pd[i].keys_read == 1) // 如果按键被释放（假设高电平表示释放）
            {
                key_pd[i].keys_flag = 1; // 设置按键标志位为1，可能表示按键已经完成一次按下释放的操作
                key_pd[i].keys_progres = 0; // 将按键进度重置为0
            }
        } break;
    }
}

	
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
void steering_ring(void)//转向�?
{              
	pid_turn=turn_PID_yaw(&yaw_pid,yawl,L_Target_Position);

	straight_right=(int16_t)pid1(&right_pid,Encoder1,L_Target_Speed+pid_turn);//
	straight_left=(int16_t)pid1(&left_pid,Encoder2,L_Target_Speed-pid_turn);//

	if(straight_right>7200)straight_right=7200;
	if(straight_left>7200)straight_left=7200;	
	if(straight_right<-7200)straight_right=-7200;
	if(straight_left<-7200)straight_left=-7200;	

	Load(straight_left,straight_right);
}


void Track_ring(void)//寻迹�?
{
	track_turn=track_pid(&track_pid_assignment,track_sum);
	
	track_left=(int16_t)pid1(&left_pid,Encoder2,20-track_turn);//
	track_right=(int16_t)pid1(&right_pid,Encoder1,20+track_turn);//
	
	
	if(track_left>7200)track_left=7200;
	if(track_right>7200)track_right=7200;	
	if(track_right<-7200)track_right=-7200;
	if(track_left<-7200)track_left=-7200;	
	
	Load(track_left,track_right);
}
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,100);
	return ch;
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
//  steering_ring(0,0);//转向�?
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
