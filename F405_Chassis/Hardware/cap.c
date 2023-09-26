#include "cap.h"

/**
 * @brief  超级电容IO初始化
 */
void ChargeIO_Configuration(void)
{
	GPIO_InitTypeDef gpio_init;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);
	//充电使能
	gpio_init.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	gpio_init.GPIO_Mode=GPIO_Mode_OUT;
	gpio_init.GPIO_Speed=GPIO_Low_Speed;
	GPIO_Init(GPIOA,&gpio_init);	
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);		//初始化时关闭充电
	GPIO_SetBits(GPIOA,GPIO_Pin_7);	        //拉高Diode Mode
    //放电选择使能
	gpio_init.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5;
	gpio_init.GPIO_Mode=GPIO_Mode_OUT;
	gpio_init.GPIO_Speed=GPIO_Low_Speed;
	GPIO_Init(GPIOC,&gpio_init);	
	Bat_on;
	CAP_off;
}

