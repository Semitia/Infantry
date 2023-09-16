#ifndef __PID_H
#define __PID_H

/**
 * @brief  PID�ṹ��
*/
typedef struct __PID_t{
		float SetPoint;			//�趨Ŀ��ֵ
		float SetPointLast;
		float deadband; //����
		
		float P;						//��������
		float I;						//���ֳ���
		float D;						//΢�ֳ���
		
		float LastError;		//ǰ�����
		float PreError;			//��ǰ���
		float SumError;			//�������
		float dError;
		
		float ErrorMax;			//ƫ������ ����ƫ���򲻼����������
		float IMax;					//��������
		
		float POut;					//�������
		float IOut;					//�������
		float DOut;					//΢�����
	  float OutMax;       //�޷�
}Pid_t;

float PID_Calc(Pid_t * P, float target, float ActualValue);
#endif
