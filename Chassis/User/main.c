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
	TIM4_Init();
}

/**
  * @brief  ∫¡√Î—” ±
  * @param  microseconds to delay
  */
void delay_ms(unsigned int t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=10300;
		while(a--);
	}
}
