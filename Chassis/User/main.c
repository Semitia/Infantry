#include "main.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

void System_Init(void);
void System_Configration(void);
/**
  * @brief  Main Function
  */
int main()
{
	System_Configration();
	System_Init();

	delay_ms(100);
	startTast();
	vTaskStartScheduler();
	
	while(1){}
} 
/**
  * @brief  Configuration 
  */
void System_Configration(void)
{

}

void System_Init(void)
{
	// while(SysTick_Config(168000));	
	// SuperPower_Configuration();//ADC��ʼ��Ҫ���ڴ���֮ǰ����Ȼ���ڲ��ܽ�������
	// DAC1_Init();
	// CAN1_Configuration();
	// CAN2_Configuration();
	// VOFA_USART_Configuration();
	// UART4_Configuration();
	// TIM2_Configuration();
	// TIM4_Configuration();
	// COUNTER_Configuration();
	// IWDG_Config(IWDG_Prescaler_64 ,625);
	// i2c_init();
	// ChargeIO_Configuration();
	// Charge_Off;
	// ChargeState = ChargeOff;
	// delay_ms(300);//�����ذ��ʼ����ɣ���ֹ���ذ��ʼ������������̷��ʹ�������
}

