 #include "vofa.h"

 extern PID_Controller yaw_pid;
 extern PID_Controller right_pid;

 extern int16_t L_Target_Position;
 extern int16_t L_Target_Speed;

 //uint8_t RxBuffer[1];//���ڽ��ջ���
 //uint16_t RxLine = 0;//ָ���
 uint8_t DataBuff[200];//ָ������
 
  
 float Get_Data(void)
 {
     uint8_t data_Start_Num = 0; // ��¼����λ��ʼ�ĵط�
     uint8_t data_End_Num = 0; // ��¼����λ�����ĵط�
     uint8_t data_Num = 0; // ��¼����λ��
     uint8_t minus_Flag = 0; // �ж��ǲ��Ǹ���
     float data_return = 0; // �����õ�������
     for(uint8_t i=0;i<200;i++) // ���ҵȺź͸�̾�ŵ�λ��
     {
         if(DataBuff[i] == '=') data_Start_Num = i + 1; // +1��ֱ�Ӷ�λ��������ʼλ
         if(DataBuff[i] == '!')
         {
             data_End_Num = i - 1;
             break;
         }
     }
     if(DataBuff[data_Start_Num] == '-') // ����Ǹ���
     {
         data_Start_Num += 1; // ����һλ������λ
         minus_Flag = 1; // ����flag
     }
     data_Num = data_End_Num - data_Start_Num + 1;
     if(data_Num == 4) // ���ݹ�4λ
     {
         data_return = (DataBuff[data_Start_Num]-48)  + (DataBuff[data_Start_Num+2]-48)*0.1f +
                 (DataBuff[data_Start_Num+3]-48)*0.01f;
     }
     else if(data_Num == 5) // ���ݹ�5λ
     {
         data_return = (DataBuff[data_Start_Num]-48)*10 + (DataBuff[data_Start_Num+1]-48) + (DataBuff[data_Start_Num+3]-48)*0.1f +
                 (DataBuff[data_Start_Num+4]-48)*0.01f;
     }
     else if(data_Num == 6) // ���ݹ�6λ
     {
         data_return = (DataBuff[data_Start_Num]-48)*100 + (DataBuff[data_Start_Num+1]-48)*10 + (DataBuff[data_Start_Num+2]-48) +
                 (DataBuff[data_Start_Num+4]-48)*0.1f + (DataBuff[data_Start_Num+5]-48)*0.01f;
     }
     if(minus_Flag == 1)  data_return = -data_return;
 //    printf("data=%.2f\r\n",data_return);
     return data_return;
 }
 
 /*
  * ���ݴ�����Ϣ����PID����
  */
 void USART_PID_Adjust(uint8_t Motor_n)
 {
     float data_Get = Get_Data(); // ��Ž��յ�������
 //    printf("data=%.2f\r\n",data_Get);e
     if(Motor_n == 1)//��ߵ��
     {
         if(DataBuff[0]=='P' && DataBuff[1]=='1') // λ�û�P
             yaw_pid.p = data_Get;
         else if(DataBuff[0]=='I' && DataBuff[1]=='1') // λ�û�I
             yaw_pid.i = data_Get;
         else if(DataBuff[0]=='D' && DataBuff[1]=='1') // λ�û�D
             yaw_pid.d = data_Get;
         else if(DataBuff[0]=='P' && DataBuff[1]=='2') // �ٶȻ�P
             right_pid.p = data_Get;
         else if(DataBuff[0]=='I' && DataBuff[1]=='2') // �ٶȻ�I
             right_pid.i = data_Get;
         else if(DataBuff[0]=='D' && DataBuff[1]=='2') // �ٶȻ�D
             right_pid.d = data_Get;
         else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //Ŀ���ٶ�
             L_Target_Speed = data_Get;
         else if((DataBuff[0]=='P' && DataBuff[1]=='o') && DataBuff[2]=='s') //Ŀ��λ��
             L_Target_Position = data_Get;
     }
//    else if(Motor_n == 0) // �ұߵ��
//    {
//        if(DataBuff[0]=='P' && DataBuff[1]=='1') // λ�û�P
//            pid_r_position.kp = data_Get;
//        else if(DataBuff[0]=='I' && DataBuff[1]=='1') // λ�û�I
//            pid_r_position.ki = data_Get;
//        else if(DataBuff[0]=='D' && DataBuff[1]=='1') // λ�û�D
//            pid_r_position.kd = data_Get;
//        else if(DataBuff[0]=='P' && DataBuff[1]=='2') // �ٶȻ�P
//            pid_r_speed.kp = data_Get;
//        else if(DataBuff[0]=='I' && DataBuff[1]=='2') // �ٶȻ�I
//            pid_r_speed.ki = data_Get;
//        else if(DataBuff[0]=='D' && DataBuff[1]=='2') // �ٶȻ�D
//            pid_r_speed.kd = data_Get;
//        else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //Ŀ���ٶ�
//            R_Target_Speed = data_Get;
//        else if((DataBuff[0]=='P' && DataBuff[1]=='o') && DataBuff[2]=='s') //Ŀ��λ��
//            R_Target_Position = data_Get;
//    }
 }
 void DL_Serial_SendArray(UART_Regs *port,uint8_t *buffer, uint16_t len)
{
  while(len--)
  {
    DL_UART_Main_transmitDataBlocking(port, *buffer);
    buffer++;
  }
}
 void SendDataToVOFA(float target, int16_t speed,int16_t speed1) 
{
    char buffer[32];  // ���ڴ洢Ҫ���͵��ַ���
    sprintf(buffer, "%d,%d,%f\r\n", speed,speed1, target);  // ��ʽ���ַ���
    DL_Serial_SendArray(UART_1_INST, (uint8_t*)buffer, strlen(buffer));  // �������ݵ�VOFA
}

void SendDataToVOFA_two(int16_t target,float yaw ) 
{
   char buffer[32];  // ���ڴ洢Ҫ���͵��ַ���
   sprintf(buffer, "%d,%f\r\n",target,yaw);  // ��ʽ���ַ���
   DL_Serial_SendArray(UART_1_INST, (uint8_t*)buffer, strlen(buffer));  // �������ݵ�VOFA
}





