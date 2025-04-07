#ifndef __PID_H
#define __PID_H


typedef struct{
	float kp;					//比例参数
	float ki;					//积分参数
	float	kd;					//微分参数
	float error;			//偏差值
	float	expect;			//期望值
	float	feedback;		//反馈值
	float	last_error;	//上次偏差
	float dis_error;	//微分项
	float	integral;		//积分控制器输出
	float output;			//当前控制器输出
	float last_output;//上次控制器输出
	float error_backup[20];			    //历史偏差值
	uint8_t dis_error_gap_cnt;
  uint8_t error_limit_flag;				//偏差限幅标志位
	float error_limit_max;					//最大偏差限幅值
	uint8_t integral_separate_flag;	//积分分离标志位
	float integral_separate_limit;	//积分分离偏差值
	float integral_limit_max;				//最大积分限幅值
	float output_limit_max;             //控制器总输出限幅值
	uint8_t init_flag;
	float dis_error_lpf;
	lpf_param lpf_params;
	lpf_buf lpf_buffer;
	systime _time;
}controller;


typedef struct PID {
        float  Kp;         //  Proportional Const  P??
        float  Ki;           //  Integral Const      I??
        float  Kd;         //  Derivative Const    D??

        float  PrevError ;          //  Error[-2]
        float  LastError;          //  Error[-1]
        float  Error;              //  Error[0 ]
        float  DError;            //pid->Error - pid->LastError
        float  SumError;           //  Sums of Errors

        float  output;

        float  Integralmax;      //???????
        float  outputmax;        //???????
} PID;



void PID_Init(PID *pid, float Kp , float Ki , float Kd , float Limit_value);  

float PID_Position_Calc(PID *pid, float Target_val, float Actual_val);  //???PID

float abs_limit(float value, float ABS_MAX);   //????,??????

float PID_Incremental_Calc(PID *pid, float Target_val, float Actual_val);

float guding_angle( PID *pid,float TAL_angle,float AVL_angle);


float pid_control_run(controller *ctrl);
void  pid_integrate_reset(controller *ctrl);

float pid_control_dt_run(controller *ctrl,float dt);
float pid_control_dt_yaw_run(controller *ctrl,float dt);

void pid_control_init(controller *ctrl,
											float kp,
											float ki,
											float kd,
											float err_max,
											float integral_max,
											float output_max,
											uint8_t err_limit_flag,
											uint8_t integral_separate_flag,
											float integral_separate_limit,
											uint8_t dis_error_gap_cnt);
											
#endif




















