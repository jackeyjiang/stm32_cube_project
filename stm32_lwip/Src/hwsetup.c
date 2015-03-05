
#include "FreeRTOS.h"
#include "trcUser.h"
#include <stm32f4xx.h>

void prvSetupHardware( void );

void prvSetupHardware( void )
{
}

long Tim1RunCounter;


extern void MX_TIM1_Init(void);	
void vConfigureTimerForRunTimeStats( void )
{
	/* init variables */
	Tim1RunCounter=100;
  MX_TIM1_Init();
}


unsigned ValueTimerForRunTimeStats(void){
	return Tim1RunCounter ;
}

