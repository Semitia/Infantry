#include "main.h"

#define CAP_MAX_W      7000
#define Rand_S         0.5f   //���ڳ���
#define RandThreshold  0.4f   //ֱ��ƫ��
#define RANDA          1.3f   //���ҷ�ֵ




/**********************************************************************************************************
*�� �� ��: Chassis_CurrentPid_Cal
*����˵��: ���̲���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Chassis_Loop_Cal(void)
{
	int i=0;
	Chassis_Speed_Cal();//����xyw�����ٶ�
	motion_decomposition(chassis.Velocity_vectors);//�˶��ֽ⣬����ÿ�����ӵ�setpoint
	
	//�������ƣ�����ÿ�����ӵĵ���ֵ
	if(F405.Chassis_Flag == Chassis_Powerdown_Mode)
	{
				
		for(i=0;i<4;i++)
		{
			chassis.WheelCurrentSend[i] = 0;
			chassis.SteeringVolSend[i] = 0;
		}
		powerlimit.set_superpower = JudgeReceive.MaxPower;//ȫ�����
	}
	else
	{
		for(int i=0;i<4;i++){
//			powerlimit.pidChassisWheelSpeed[i].SetPoint = chassis.Velocity_vectors[i].v;
//			powerlimit.Speed_Now[i] = chassis.ChassisMotorCanReceive[i].RealSpeed;
			if((chassis.ChassisMotorCanReceive[i].Current != powerlimit.Current_Last[i]))//��ε���ֵ���ϴβ�һ����
			{
				powerlimit.Motor_lost[i] = 0;
				powerlimit.pidChassisWheelSpeed[i].SetPoint = chassis.Velocity_vectors[i].v;
			}
			else{
				powerlimit.Motor_lost[i] = 1;
				powerlimit.pidChassisWheelSpeed[i].SetPoint = chassis.ChassisMotorCanReceive[i].RealSpeed;
			}	
			powerlimit.Current_Last[i] = chassis.ChassisMotorCanReceive[i].Current;
			powerlimit.Speed_Now[i] = chassis.ChassisMotorCanReceive[i].RealSpeed;
		}		
		
		//��������
		powerlimit.actual_ina260_power = INA260_1.Power/1000;
		if(JudgeReceive.MaxPower<150 && JudgeReceive.MaxPower>0){
			powerlimit.referee_max_power = JudgeReceive.MaxPower;
		}else{
			powerlimit.referee_max_power = 200;
		}
		powerlimit.remainEnergy = JudgeReceive.remainEnergy;
		powerlimit.capEnergy = 0.5*CAP_C*(superpower.actual_vol*superpower.actual_vol);
		
		if(F405.SuperPowerLimit==CAP && superpower.actual_vol > superpower.CAP_AD_H){//�����㹻��ȫ�õ���
			powerlimit.PowerState_control = CAP;
		}else{
			powerlimit.PowerState_control = HalfCAP;
		}	
//        powerlimit.PowerState_control = BAT;
		
		PowerLimit(&powerlimit,superpower.PowerState);//������������ֵ ��ȷ��������������ϵͳĿ��
		
		set_powersate(&powerlimit,&superpower,powerlimit.PowerState_control);//�����������ݿ����ź�superpower.PowerState_Set
	
		
		for(int i=0;i<4;i++){
			chassis.WheelCurrentSend[i] = powerlimit.Current_set[i];
		}
	}
	
	//���͵���
	Chassis_Current_send();
	
}

/**********************************************************************************************************
*�� �� ��: BuildF105
*����˵��: ����Ҫ�����ϲ���F105�ṹ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void BuildF105(void)
{
	
	if(JudgeReceive.robot_id < 10)
		F105.Sendmessage.RobotRed = 1;
	else
		F105.Sendmessage.RobotRed = 0;			//0Ϊ��ɫ��1Ϊ��ɫ
	switch(JudgeReceive.BulletSpeedMax17)
	{
		case 15:
		{
			F105.Sendmessage.BulletSpeedLevel = 0;
			break;
		}
		case 18:
		{
			F105.Sendmessage.BulletSpeedLevel = 1;
			break;
		}
		case 30:
		{
			F105.Sendmessage.BulletSpeedLevel = 2;
			break;
		}
		default:
		{
			F105.Sendmessage.BulletSpeedLevel = 0;
			break;
		}
	}
	F105.Sendmessage.shooterHeat17 = JudgeReceive.shooterHeat17;
	F105.Sendmessage.HeatMax17 = JudgeReceive.HeatMax17;
	F105.Sendmessage.HeatCool17 = (unsigned char)(JudgeReceive.HeatCool17);
	F105.Sendmessage.RobotLevel = JudgeReceive.RobotLevel;
	
	F105.ChassisSpeedw=0.026f*(chassis.ChassisMotorCanReceive[0].RealSpeed+chassis.ChassisMotorCanReceive[1].RealSpeed+chassis.ChassisMotorCanReceive[2].RealSpeed+chassis.ChassisMotorCanReceive[3].RealSpeed);
}



/**********************************************************************************************************
*�� �� ��: Chassis_Init
*����˵��: �������˶�������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Chassis_Init(void)
{	
	ZeroCheck_Init();
	powerlimit.k_dynamic = RM3508_K;
#if Robot_ID  == 46  
	powerlimit.error_control = 5; //�����5w
	powerlimit.No_limited_Power = 1000; 		//W �������������ƹ���
	
	powerlimit.Min_remainEnergy = 25;  		//15J ���޸�
	powerlimit.Add_HalfCAP_Power = 10;			//W �������ݶ��ܵĹ���
	powerlimit.Min_capVol = 16.5;			//V  �������ݱ���������������
	
	//������С��������
	powerlimit.Min_capEnergy = 0.5*CAP_C*(powerlimit.Min_capVol*powerlimit.Min_capVol);
#else
	powerlimit.error_control = 5; //�����5W
	powerlimit.No_limited_Power = 1000; 		//W �������������ƹ���
	
	powerlimit.Min_remainEnergy = 20;  		//15J ���޸�
	powerlimit.Add_HalfCAP_Power = 10;			//W �������ݶ��ܵĹ���
	powerlimit.Min_capVol = 16.5;			//V  �������ݱ���������������
	
	
	//������С��������
	powerlimit.Min_capEnergy = 0.5*CAP_C*(powerlimit.Min_capVol*powerlimit.Min_capVol);	
#endif
	short i=0;
	for(i=0;i<4;i++)
	{
		//�����������Ƴ���ת��
		powerlimit.pidChassisWheelSpeed[i].deadband=0.0;
		powerlimit.pidChassisWheelSpeed[i].P = 20.0f;
		powerlimit.pidChassisWheelSpeed[i].I = 1.0f;		
		powerlimit.pidChassisWheelSpeed[i].D = 0.0f;
		powerlimit.pidChassisWheelSpeed[i].ErrorMax = 1000.0f;
		powerlimit.pidChassisWheelSpeed[i].IMax = 2500.0f;
		powerlimit.pidChassisWheelSpeed[i].SetPoint = 0.0f;	
		powerlimit.pidChassisWheelSpeed[i].OutMax = 16000.0f;
		
		//������
		chassis.PIDSteeringAnglePosition[i].deadband=2.5;
		chassis.PIDSteeringAnglePosition[i].P=7.5f;
		chassis.PIDSteeringAnglePosition[i].I=0.0f;
		chassis.PIDSteeringAnglePosition[i].D=0.0f;
		chassis.PIDSteeringAnglePosition[i].IMax=1000.0f;
		chassis.PIDSteeringAnglePosition[i].SetPoint=0.0f;
		chassis.PIDSteeringAnglePosition[i].ErrorMax = 1000.0f;
		chassis.PIDSteeringAnglePosition[i].OutMax=4000.0f;
		
		chassis.PIDSteeringAngleSpeed[i].deadband=0.0;
		chassis.PIDSteeringAngleSpeed[i].P=120.0f;
		chassis.PIDSteeringAngleSpeed[i].I=0.2f; 
		chassis.PIDSteeringAngleSpeed[i].D=0.0f;
		chassis.PIDSteeringAngleSpeed[i].IMax=16000.0f;
		chassis.PIDSteeringAngleSpeed[i].SetPoint=0.0f;
		chassis.PIDSteeringAngleSpeed[i].ErrorMax = 8000;
		chassis.PIDSteeringAngleSpeed[i].OutMax=GM6020_MAX_VOLTAGE;
		
		chassis.Velocity_vectors[i].v = 0;
		chassis.Velocity_vectors_last[i].v = 0;
		//ChassisSteeringPIDRUN_Send(Wheel);
	}

	chassis.k_xy = 3.0f;
	chassis.k_w = 1.5f;
	chassis.Steeringwheel_Mode = Stright;
	chassis.sgn = 1.0;
	
	//��ʼ�Ƕ�
	chassis.SteeringAnglePosition_init[0] = 5147;
	chassis.SteeringAnglePosition_init[1] = 3061;
	chassis.SteeringAnglePosition_init[2] = 4921;
	chassis.SteeringAnglePosition_init[3] = 326;
}
/**********************************************************************************************************
*�� �� ��: Chassis_task
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t Chassis_high_water;

extern uint8_t JudgeReveice_Flag;
//extern TaskHandle_t JudgeReceiveTask_Handler; //������
extern TaskHandle_t User_Tasks[TASK_NUM];
uint32_t testtask=0;
void Chassis_task(void *pvParameters)
{
	portTickType xLastWakeTime;   //config=0. 32v bits
#if Mecanum == 2
	const portTickType xFrequency = 2;
#else
	const portTickType xFrequency = 1;
#endif
	Chassis_Init();
	
	vTaskDelay(100);
	while (1) {
		xLastWakeTime = xTaskGetTickCount();
		
		if(JudgeReveice_Flag)
		{
		 xTaskNotifyGive(User_Tasks[JUDGERECEIVE_TASK]);
		}
		
		//���ݳ�ŵ����
		if(JudgeReceive.remainEnergy<20)//ʡ�����������ˣ��������ݳ����
		{
		Charge_Off;
		ChargeState = ChargeOff ;
		}
		else
		{
			Charge_On;
			ChargeState = ChargeOn;
		}	
		//��������
		Chassis_Loop_Cal();
		BuildF105();
		Can2Send0(&F105);
		
		VOFA_Send();

		IWDG_Feed();//ι��		
		vTaskDelayUntil(&xLastWakeTime,xFrequency); 
		 
	#if INCLUDE_uxTaskGetStackHighWaterMark
		Chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
	#endif
	}
}
