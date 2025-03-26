 #include "vofa.h"

 extern PID_Controller yaw_pid;
 extern PID_Controller right_pid;

 extern int16_t L_Target_Position;
 extern int16_t L_Target_Speed;

 //uint8_t RxBuffer[1];//串口接收缓冲
 uint16_t RxLine = 0;//指令长度
 uint8_t DataBuff[200];//指令内容
 
  
 float Get_Data(void)
 {
     uint8_t data_Start_Num = 0; // 记录数据位开始的地方
     uint8_t data_End_Num = 0; // 记录数据位结束的地方
     uint8_t data_Num = 0; // 记录数据位数
     uint8_t minus_Flag = 0; // 判断是不是负数
     float data_return = 0; // 解析得到的数据
     for(uint8_t i=0;i<200;i++) // 查找等号和感叹号的位置
     {
         if(DataBuff[i] == '=') data_Start_Num = i + 1; // +1是直接定位到数据起始位
         if(DataBuff[i] == '!')
         {
             data_End_Num = i - 1;
             break;
         }
     }
     if(DataBuff[data_Start_Num] == '-') // 如果是负数
     {
         data_Start_Num += 1; // 后移一位到数据位
         minus_Flag = 1; // 负数flag
     }
     data_Num = data_End_Num - data_Start_Num + 1;
     if(data_Num == 4) // 数据共4位
     {
         data_return = (DataBuff[data_Start_Num]-48)  + (DataBuff[data_Start_Num+2]-48)*0.1f +
                 (DataBuff[data_Start_Num+3]-48)*0.01f;
     }
     else if(data_Num == 5) // 数据共5位
     {
         data_return = (DataBuff[data_Start_Num]-48)*10 + (DataBuff[data_Start_Num+1]-48) + (DataBuff[data_Start_Num+3]-48)*0.1f +
                 (DataBuff[data_Start_Num+4]-48)*0.01f;
     }
     else if(data_Num == 6) // 数据共6位
     {
         data_return = (DataBuff[data_Start_Num]-48)*100 + (DataBuff[data_Start_Num+1]-48)*10 + (DataBuff[data_Start_Num+2]-48) +
                 (DataBuff[data_Start_Num+4]-48)*0.1f + (DataBuff[data_Start_Num+5]-48)*0.01f;
     }
     if(minus_Flag == 1)  data_return = -data_return;
 //    printf("data=%.2f\r\n",data_return);
     return data_return;
 }
 
 /*
  * 根据串口信息进行PID调参
  */
 void USART_PID_Adjust(uint8_t Motor_n)
 {
     float data_Get = Get_Data(); // 存放接收到的数据
 //    printf("data=%.2f\r\n",data_Get);e
     if(Motor_n == 1)//左边电机
     {
         if(DataBuff[0]=='P' && DataBuff[1]=='1') // 位置环P
             yaw_pid.p = data_Get;
         else if(DataBuff[0]=='I' && DataBuff[1]=='1') // 位置环I
             yaw_pid.i = data_Get;
         else if(DataBuff[0]=='D' && DataBuff[1]=='1') // 位置环D
             yaw_pid.d = data_Get;
         else if(DataBuff[0]=='P' && DataBuff[1]=='2') // 速度环P
             right_pid.p = data_Get;
         else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 速度环I
             right_pid.i = data_Get;
         else if(DataBuff[0]=='D' && DataBuff[1]=='2') // 速度环D
             right_pid.d = data_Get;
         else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //目标速度
             L_Target_Speed = data_Get;
         else if((DataBuff[0]=='P' && DataBuff[1]=='o') && DataBuff[2]=='s') //目标位置
             L_Target_Position = data_Get;
     }
 }

 void HAL_Serial_SendArray(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t len)
{
    HAL_UART_Transmit(huart, buffer, len, HAL_MAX_DELAY);
}

 void SendDataToVOFA(float target, int16_t speed,int16_t speed1) 
{
    char buffer[32];  // 用于存储要发送的字符串
    sprintf(buffer, "%d,%d,%f\r\n", speed,speed1, target);  // 格式化字符串
    HAL_Serial_SendArray(&huart1, (uint8_t*)buffer, strlen(buffer));  // 发送数据到VOFA
}

void SendDataToVOFA_two(int16_t target,float yaw ) 
{
   char buffer[32];  // 用于存储要发送的字符串
   sprintf(buffer, "%d,%f\r\n",target,yaw);  // 格式化字符串
   HAL_Serial_SendArray(&huart1, (uint8_t*)buffer, strlen(buffer));  // 发送数据到VOFA
}


// /*
//  * 解析出DataBuff中的数据
//  * 返回解析得到的数据
//  */
// extern PID_Controller yaw_pid;
// extern PID_Controller right_pid;

// uint8_t RxBuffer;//串口接收缓冲
// uint16_t RxLine = 0;//指令长度
// char  DataBuff[256];//指令内容
// //extern uint8_t Uart1_Rx_Cnt = 0;		//接收缓冲计数
// // char RxBuffer[256]; 
// // uint8_t aRxBuffer;			//接收中断缓冲
// // uint8_t Uart1_Rx_Cnt = 0;		//接收缓冲计数
// extern int16_t L_Target_Position;
// extern int16_t L_Target_Speed;

 
// float Get_Data(void)
// {
//     uint8_t data_Start_Num = 0; // 记录数据位开始的地方
//     uint8_t data_End_Num = 0; // 记录数据位结束的地方
//     uint8_t data_Num = 0; // 记录数据位数
//     uint8_t minus_Flag = 0; // 判断是不是负数
//     float data_return = 0; // 解析得到的数据
//     for(uint8_t i=0;i<200;i++) // 查找等号和感叹号的位置
//     {
//         if(DataBuff[i] == '=') data_Start_Num = i + 1; // +1是直接定位到数据起始位
//         if(DataBuff[i] == '!')
//         {
//             data_End_Num = i - 1;
//             break;
//         }
//     }
//     if(DataBuff[data_Start_Num] == '-') // 如果是负数
//     {
//         data_Start_Num += 1; // 后移一位到数据位
//         minus_Flag = 1; // 负数flag
//     }
//     data_Num = data_End_Num - data_Start_Num + 1;
//     if(data_Num == 4) // 数据共4位
//     {
//         data_return = (DataBuff[data_Start_Num]-48)  + (DataBuff[data_Start_Num+2]-48)*0.1f +
//                 (DataBuff[data_Start_Num+3]-48)*0.01f;
//     }
//     else if(data_Num == 5) // 数据共5位
//     {
//         data_return = (DataBuff[data_Start_Num]-48)*10 + (DataBuff[data_Start_Num+1]-48) + (DataBuff[data_Start_Num+3]-48)*0.1f +
//                 (DataBuff[data_Start_Num+4]-48)*0.01f;
//     }
//     else if(data_Num == 6) // 数据共6位
//     {
//         data_return = (DataBuff[data_Start_Num]-48)*100 + (DataBuff[data_Start_Num+1]-48)*10 + (DataBuff[data_Start_Num+2]-48) +
//                 (DataBuff[data_Start_Num+4]-48)*0.1f + (DataBuff[data_Start_Num+5]-48)*0.01f;
//     }
//     if(minus_Flag == 1)  data_return = -data_return;
// //    printf("data=%.2f\r\n",data_return);
//     return data_return;
// }

// /*
//  * 根据串口信息进行PID调参
//  */
// void USART_PID_Adjust(uint8_t Motor_n)
// {
//     float data_Get = Get_Data(); // 存放接收到的数据
// //    printf("data=%.2f\r\n",data_Get);e
//     if(Motor_n == 1)//左边电机
//     {
//         if(DataBuff[0]=='P' && DataBuff[1]=='1') // 位置环P
//             yaw_pid.p = data_Get;
//         else if(DataBuff[0]=='I' && DataBuff[1]=='1') // 位置环I
//             yaw_pid.i = data_Get;
//         else if(DataBuff[0]=='D' && DataBuff[1]=='1') // 位置环D
//             yaw_pid.d = data_Get;
//         else if(DataBuff[0]=='P' && DataBuff[1]=='2') // 速度环P
//             right_pid.p = data_Get;
//         else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 速度环I
//             right_pid.i = data_Get;
//         else if(DataBuff[0]=='D' && DataBuff[1]=='2') // 速度环D
//             right_pid.d = data_Get;
//         else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //目标速度
//             L_Target_Speed = data_Get;
//         else if((DataBuff[0]=='P' && DataBuff[1]=='o') && DataBuff[2]=='s') //目标位置
//             L_Target_Position = data_Get;
//     }
// //    else if(Motor_n == 0) // 右边电机
// //    {
// //        if(DataBuff[0]=='P' && DataBuff[1]=='1') // 位置环P
// //            pid_r_position.kp = data_Get;
// //        else if(DataBuff[0]=='I' && DataBuff[1]=='1') // 位置环I
// //            pid_r_position.ki = data_Get;
// //        else if(DataBuff[0]=='D' && DataBuff[1]=='1') // 位置环D
// //            pid_r_position.kd = data_Get;
// //        else if(DataBuff[0]=='P' && DataBuff[1]=='2') // 速度环P
// //            pid_r_speed.kp = data_Get;
// //        else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 速度环I
// //            pid_r_speed.ki = data_Get;
// //        else if(DataBuff[0]=='D' && DataBuff[1]=='2') // 速度环D
// //            pid_r_speed.kd = data_Get;
// //        else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //目标速度
// //            R_Target_Speed = data_Get;
// //        else if((DataBuff[0]=='P' && DataBuff[1]=='o') && DataBuff[2]=='s') //目标位置
// //            R_Target_Position = data_Get;
// //    }
// }

// extern uint8_t get_data;
// // void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// // {
// //   /* Prevent unused argument(s) compilation warning */
// //   UNUSED(huart);
// //   /* NOTE: This function Should not be modified, when the callback is needed,
// //            the HAL_UART_TxCpltCallback could be implemented in the user file
// //    */
 
// // 	if(Uart1_Rx_Cnt >= 255)  //溢出判断
// // 	{
// // 		Uart1_Rx_Cnt = 0;
// // 		memset(RxBuffer,0x00,sizeof(RxBuffer));
// // 		HAL_UART_Transmit(&huart1, (uint8_t *)"数据溢出", 10,0xFFFF); 	
        
// // 	}
// // 	else
// // 	{
// // 		RxBuffer[Uart1_Rx_Cnt++] = aRxBuffer;   //接收数据转存
	
// // 		if((RxBuffer[Uart1_Rx_Cnt-1] == 0x0A)&&(RxBuffer[Uart1_Rx_Cnt-2] == 0x0D)) //判断结束位
// // 		{
// // 			HAL_UART_Transmit(&huart1, (uint8_t *)&RxBuffer, Uart1_Rx_Cnt,0xFFFF); //将收到的信息发送出去
// //             while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);//检测UART发送结束
// // 			Uart1_Rx_Cnt = 0;
// // 			memset(RxBuffer,0x00,sizeof(RxBuffer)); //清空数组
// // 		}
// // 	}
	
// // 	HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);   //再开启接收中断
// // }
// // // 接收完成回调函数（HAL 库自动调用）
// // void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// // {
// //     if (huart == &huart1) // 判断是 USART1
// //     {
// //         // 在这里处理接收到的数据（类似第一段代码的逻辑）
// //         if(RxBuffer[0] == 0x21) // 结束标志
// //         {
// //             printf("RXLen=%d\r\n", RxLine);
// //             for(int i = 0; i < RxLine; i++)
// //                 printf("UART DataBuff[%d] = %c\r\n", i, DataBuff[i]);
            
// //             USART_PID_Adjust(1); // 数据处理
// //             memset(DataBuff, 0, sizeof(DataBuff)); // 清空缓存
// //             RxLine = 0; // 重置计数
// //         }
// //         else
// //         {
// //             DataBuff[RxLine++] = RxBuffer[0]; // 存储数据
// //         }

// //         // 重新启动接收（HAL 库需要手动重新启用接收）
// //         HAL_UART_Receive_IT(&huart1, RxBuffer, 1);
// //     }
// // }

// // void USART1_IRQHandler(void)
// // {
// // 	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == SET)		//判断是否是USART1的接收事件触发的中断
// // 	{
// // 	  		RxBuffer[0] = USART_ReceiveData(USART1);				//读取数据寄存器，存放在接收的数据变量
// // 				RxLine++;                      //每接收到一个数据，进入回调数据长度加1
// // 				DataBuff[RxLine-1]=RxBuffer[0];  //把每次接收到的数据保存到缓存数组
// // 				if(RxBuffer[0]==0x21)            //接收结束标志位，这个数据可以自定义，根据实际需求，这里只做示例使用，不一定是0x21
// // 				{
// // 						printf("RXLen=%d\r\n",RxLine);
// // 						for(int i=0;i<RxLine;i++)
// // 								printf("UART DataBuff[%d] = %c\r\n",i,DataBuff[i]);
// // 						USART_PID_Adjust(1);//数据解析和参数赋值函数
// // 						memset(DataBuff,0,sizeof(DataBuff));  //清空缓存数组
// // 						RxLine=0;  //清空接收长度
// // 				}
// // 			RxBuffer[0]=0;
			
// // 	}
	
// // }

