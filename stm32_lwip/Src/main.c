/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 08/02/2015 01:35:05
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "ff.h"
#include "ff_gen_drv.h"
#include "usbh_diskio.h" /* defines USBH_Driver as external */
#include "lwip.h"
#include "usb_host.h"
#include "main.h"
#include "app_ethernet.h"
#include "tcp_server.h"
#include "stm324x9i_eval.h"
#include "stm324x9i_eval_io.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern struct netif gnetif; /* network interface structure */
/* Semaphore to signal Ethernet Link state update */
osSemaphoreId Netif_LinkSemaphore = NULL;
/* Ethernet link thread Argument */
struct link_str link_arg;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void StartThread(void const * argument);
static void ToggleLed4(void const * argument);
static void BSP_Config(void);
static void Netif_Config(void);
extern void tcpecho_init(void);
extern void udpecho_init(void);

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

NAND_HandleTypeDef hnand1;

uint8_t USBH_DriverNum;      /* FatFS USBH part */
char USBH_Path[4];           /* USBH logical drive path */

/* USER CODE BEGIN 0 */
#include "traceDemoApp.h"
#include "trcUser.h"
#include "trcConfig.h"
#include "trcHardwarePort.h"

#define TaskHandle_t xTaskHandle
#define Timer_t xTimerHandle

// The hardware init code may redefine this. If so, it is called after the trace is started.
void (*startPeriodicInterrupt)(void);

#if (SELECTED_PORT == PORT_ARM_CortexM)

#define _PORT_INIT_EXISTS

void port_init(void);

extern void prvSetupHardware(void);

void port_init(void)
{
	prvSetupHardware();
}
#endif

#if (SELECTED_PORT == PORT_Atmel_AT91SAM7)

#define _PORT_INIT_EXISTS

void port_init(void);

/* Port specific includes */
#include "console.h"

extern void prvSetupHardware(void);

void port_init(void)
{
	prvSetupHardware();
	vStartConsoleTasks(2);
}
#endif

#if (SELECTED_PORT == PORT_Renesas_RX600)

#define _PORT_INIT_EXISTS
/* Port specific includes */
#include "iodefine.h"

void port_init(void);

void port_init(void)
{
	extern void HardwareSetup( void );
	
	/* Renesas provided CPU configuration routine. The clocks are configured in
	here. */
	HardwareSetup();
}

/* The RX port uses this callback function to configure its tick interrupt.
This allows the application to choose the tick interrupt source. */
void vApplicationSetupTimerInterrupt( void )
{
	/* Enable compare match timer 0. */
	MSTP( CMT0 ) = 0;
	
	/* Interrupt on compare match. */
	CMT0.CMCR.BIT.CMIE = 1;
	
	/* Set the compare match value. */
	CMT0.CMCOR = ( unsigned short ) ( ( ( configPERIPHERAL_CLOCK_HZ / configTICK_RATE_HZ ) -1 ) / 8 );
	
	/* Divide the PCLK by 8. */
	CMT0.CMCR.BIT.CKS = 0;
	
	/* Enable the interrupt... */
	_IEN( _CMT0_CMI0 ) = 1;
	
	/* ...and set its priority to the application defined kernel priority. */
	_IPR( _CMT0_CMI0 ) = configKERNEL_INTERRUPT_PRIORITY;
	
	/* Start the timer. */
	CMT.CMSTR0.BIT.STR0 = 1;
}

#elif (SELECTED_PORT == PORT_MICROCHIP_PIC32MX)

/* Hardware specific includes. */
#include "ConfigPerformance.h"

/* Core configuratin fuse settings */
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_2
#pragma config CP = OFF, BWP = OFF, PWP = OFF

/* Additional config fuse settings for other supported processors */
#if defined(__32MX460F512L__)
	#pragma config UPLLEN = OFF
#elif defined(__32MX795F512L__)
	#pragma config UPLLEN = OFF
	#pragma config FSRSSEL = PRIORITY_7
#endif

#define _PORT_INIT_EXISTS

/* Port specific includes */

void port_init(void);

void port_init(void)
{
	/* Configure the hardware for maximum performance. */
	vHardwareConfigurePerformance();

	/* Setup to use the external interrupt controller. */
	vHardwareUseMultiVectoredInterrupts();

	portDISABLE_INTERRUPTS();
}

#elif (SELECTED_PORT == PORT_MICROCHIP_PIC32MZ)

/* Core configuration fuse settings */
#pragma config FMIIEN = OFF, FETHIO = OFF, PGL1WAY = OFF, PMDL1WAY = OFF, IOL1WAY = OFF, FUSBIDIO = OFF
#pragma config FNOSC = SPLL, FSOSCEN = OFF, IESO = OFF, POSCMOD = EC
#pragma config OSCIOFNC = OFF, FCKSM = CSECMD, FWDTEN = OFF, FDMTEN = OFF
#pragma config DMTINTV = WIN_127_128, WDTSPGM = STOP, WINDIS= NORMAL
#pragma config WDTPS = PS1048576, FWDTWINSZ = WINSZ_25, DMTCNT = DMT31
#pragma config FPLLIDIV = DIV_3, FPLLRNG = RANGE_13_26_MHZ, FPLLICLK = PLL_POSC
#pragma config FPLLMULT = MUL_50, FPLLODIV = DIV_2, UPLLFSEL = FREQ_12MHZ, UPLLEN = OFF
#pragma config EJTAGBEN = NORMAL, DBGPER = PG_ALL, FSLEEP = OFF, FECCCON = OFF_UNLOCKED
#pragma config BOOTISA = MIPS32, TRCEN = ON, ICESEL = ICS_PGx2, JTAGEN = OFF, DEBUG = ON
#pragma config CP = OFF
#pragma config_alt FWDTEN=OFF
#pragma config_alt USERID = 0x1234u

#define _PORT_INIT_EXISTS

/* Port specific includes */

void port_init(void);

void port_init(void)
{
	/* Configure the hardware for maximum performance. */
	vHardwareConfigurePerformance();

	/* Setup to use the external interrupt controller. */
	vHardwareUseMultiVectoredInterrupts();

	portDISABLE_INTERRUPTS();
}

#elif (SELECTED_PORT == PORT_Win32)

#define _PORT_INIT_EXISTS

/* Port specific includes */

void stopHook(void);

void stopHook(void)
{
	FILE * f = fopen("out.bin", "wb");	
	fwrite(vTraceGetTraceBuffer(), uiTraceGetTraceBufferSize(), 1, f);
	printf("Stopped! Written to out.bin.\n");
	fclose(f);
	
	system("pause");
	exit(0);
}

void port_init(void);

void port_init(void)
{
	printf("FreeRTOS+Trace Demo running...\n");
	vConfigureTimerForRunTimeStats();
	vTraceSetStopHook(stopHook);
}

#endif


void vApplicationMallocFailedHook( void );

void vApplicationMallocFailedHook( void )
{
	vTraceConsoleMessage("\n\rMalloc failed!\n\r");
}

void vApplicationStackOverflowHook( TaskHandle_t *pxTask, signed char *pcTaskName );

void vApplicationStackOverflowHook( TaskHandle_t *pxTask, signed char *pcTaskName )
{
	(void)pxTask;
	(void)pcTaskName;
	vTraceConsoleMessage("\n\rStack overflow!\n\r");
}

/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void StartThread(void const * argument);
static void StartThread1(void const * argument); 
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_FSMC_Init(void);
extern void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);

int main(void)
{

  /* USER CODE BEGIN 1 */
#ifdef _PORT_INIT_EXISTS
	port_init();
#endif	

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* System interrupt init*/
  /* Sets the priority grouping field */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  MX_FSMC_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  vTraceInitTraceData();
	
	if (! uiTraceStart() )
	{
			vTraceConsoleMessage("Could not start recorder!");
	}
  /* USER CODE END 2 */

  /* Code generated for FreeRTOS */
  /* Create Start thread */
  osThreadDef(USER_Thread, StartThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadCreate (osThread(USER_Thread), NULL);
  
  /* Start scheduler */
  osKernelStart(NULL, NULL);

  /* We should never get here as control is now taken by the scheduler */

  /* USER CODE BEGIN 3 */
  /* Infinite loop */
  while (1)
  {

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

}

/* CRC init function */
void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  HAL_CRC_Init(&hcrc);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
  
  HAL_TIM_Base_Start_IT(&htim1);

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOI_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOG_CLK_ENABLE();

}

/* FSMC initialization function */
void MX_FSMC_Init(void)
{
  FSMC_NAND_PCC_TimingTypeDef ComSpaceTiming;
  FSMC_NAND_PCC_TimingTypeDef AttSpaceTiming;

  /** Perform the NAND1 memory initialization sequence
  */
  hnand1.Instance = FSMC_NAND_DEVICE;
  /* hnand1.Init */
  hnand1.Init.NandBank = FSMC_NAND_BANK2;
  hnand1.Init.Waitfeature = FSMC_NAND_PCC_WAIT_FEATURE_ENABLE;
  hnand1.Init.MemoryDataWidth = FSMC_NAND_PCC_MEM_BUS_WIDTH_8;
  hnand1.Init.EccComputation = FSMC_NAND_ECC_DISABLE;
  hnand1.Init.ECCPageSize = FSMC_NAND_ECC_PAGE_SIZE_256BYTE;
  hnand1.Init.TCLRSetupTime = 0;
  hnand1.Init.TARSetupTime = 0;
  /* hnand1.Info */
  /* ComSpaceTiming */
  ComSpaceTiming.SetupTime = 255;
  ComSpaceTiming.WaitSetupTime = 255;
  ComSpaceTiming.HoldSetupTime = 255;
  ComSpaceTiming.HiZSetupTime = 255;
  /* AttSpaceTiming */
  AttSpaceTiming.SetupTime = 255;
  AttSpaceTiming.WaitSetupTime = 255;
  AttSpaceTiming.HoldSetupTime = 255;
  AttSpaceTiming.HiZSetupTime = 255;

  HAL_NAND_Init(&hnand1, &ComSpaceTiming, &AttSpaceTiming);

}

/* USER CODE BEGIN 4 */
void vApplicationTickHook( void );
void vApplicationIdleHook( void );

void vApplicationIdleHook( void )
{
#ifdef WIN32
	/* Sleep to reduce CPU load, but don't sleep indefinitely in case there are
	tasks waiting to be terminated by the idle task. */
	Sleep( 5 );
#endif
}

void vApplicationTickHook( void )
{
}

#if (SELECTED_PORT == PORT_MICROCHIP_PIC32MX)

void _general_exception_handler( unsigned long ulCause, unsigned long ulStatus )
{
	/* This overrides the definition provided by the kernel. Other exceptions
	should be handled here. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vAssertCalled( const char * pcFile, unsigned long ulLine )
{
volatile unsigned long ul = 0;

	( void ) pcFile;
	( void ) ulLine;

	__asm volatile( "di" );
	{
		/* Set ul to a non-zero value using the debugger to step out of this
		function. */
		while( ul == 0 )
		{
			portNOP();
		}
	}
	__asm volatile( "ei" );
}

#elif (SELECTED_PORT == PORT_MICROCHIP_PIC32MZ)

void vAssertCalled( const char * pcFile, unsigned long ulLine )
{
volatile char *pcFileName;
volatile unsigned long ulLineNumber;

	/* Prevent things that are useful to view in the debugger from being
	optimised away. */
	pcFileName = ( char * ) pcFile;
	( void ) pcFileName;
	ulLineNumber = ulLine;

	/* Set ulLineNumber to 0 in the debugger to break out of this loop and
	return to the line that triggered the assert. */
	while( ulLineNumber != 0 )
	{
		__asm volatile( "NOP" );
		__asm volatile( "NOP" );
		__asm volatile( "NOP" );
		__asm volatile( "NOP" );
		__asm volatile( "NOP" );
	}
}
/*-----------------------------------------------------------*/

/* This function overrides the normal _weak_ generic handler. */
void _general_exception_handler(void)
{
static enum {
	EXCEP_IRQ = 0, 	/* interrupt */
	EXCEP_AdEL = 4, /* address error exception (load or ifetch) */
	EXCEP_AdES, 	/* address error exception (store) */
	EXCEP_IBE, 		/* bus error (ifetch) */
	EXCEP_DBE, 		/* bus error (load/store) */
	EXCEP_Sys, 		/* syscall */
	EXCEP_Bp, 		/* breakpoint */
	EXCEP_RI, 		/* reserved instruction */
	EXCEP_CpU, 		/* coprocessor unusable */
	EXCEP_Overflow,	/* arithmetic overflow */
	EXCEP_Trap, 	/* trap (possible divide by zero) */
	EXCEP_IS1 = 16,	/* implementation specfic 1 */
	EXCEP_CEU, 		/* CorExtend Unuseable */
	EXCEP_C2E 		/* coprocessor 2 */
} _excep_code;

static unsigned long _epc_code;
static unsigned long _excep_addr;

	asm volatile( "mfc0 %0,$13" : "=r" (_epc_code) );
	asm volatile( "mfc0 %0,$14" : "=r" (_excep_addr) );

	_excep_code = ( _epc_code & 0x0000007C ) >> 2;

	for( ;; )
	{
		/* Examine _excep_code to identify the type of exception. Examine
		_excep_addr to find the address that caused the exception */
		LATHSET = 0x0007;
		Nop();
		Nop();
		Nop();
	}
}
#else

void vAssertCalled( unsigned long ulLine, const char * const pcFileName );

void vAssertCalled( unsigned long ulLine, const char * const pcFileName )
{	
	(void)ulLine;
	(void)pcFileName;

	taskDISABLE_INTERRUPTS();
	for( ;; );
}

#endif

extern void trcDemoTaskSensor;
/* USER CODE END 4 */

static void StartThread(void const * argument) {

  /* Initialize LCD and LEDs */
  BSP_Config();
  
  /*## FatFS: Link the USBH disk I/O driver ###############################*/
  USBH_DriverNum = FATFS_LinkDriver(&USBH_Driver, USBH_Path);
                             
  /* init code for LWIP */
  MX_LWIP_Init();

  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  
    /* Initilaize the LwIP stack */
  Netif_Config();
  
  /* Initialize tcp echo server */
  tcpecho_init();

  /* Initialize udp echo server */
  udpecho_init();
  
  /* Notify user about the netwoek interface config */
  User_notification(&gnetif);
  
  /* Start toogleLed4 task : Toggle LED4  every 250ms */
  osThreadDef(LED4, ToggleLed4, osPriorityLow, 0, configMINIMAL_STACK_SIZE);
  osThreadCreate(osThread(LED4), NULL);

  /* USER CODE BEGIN 5 */
  osThreadDef(USER_Thread1, StartThread1, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadCreate (osThread(USER_Thread1), NULL);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }

  /* USER CODE END 5 */ 

}

/**
  * @brief  Initializes the lwIP stack
  * @param  None
  * @retval None
  */
static void Netif_Config(void)
{
  struct ip_addr ipaddr;
  struct ip_addr netmask;
  struct ip_addr gw;	
  
  /* IP address setting */
  IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
  IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
  IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
  
  /* - netif_add(struct netif *netif, struct ip_addr *ipaddr,
  struct ip_addr *netmask, struct ip_addr *gw,
  void *state, err_t (* init)(struct netif *netif),
  err_t (* input)(struct pbuf *p, struct netif *netif))
  
  Adds your network interface to the netif_list. Allocate a struct
  netif and pass a pointer to this structure as the first argument.
  Give pointers to cleared ip_addr structures when using DHCP,
  or fill them with sane numbers otherwise. The state pointer may be NULL.
  
  The init function pointer must point to a initialization function for
  your ethernet netif interface. The following code illustrates it's use.*/
  
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);
  
  /*  Registers the default network interface. */
  netif_set_default(&gnetif);
  
  if (netif_is_link_up(&gnetif))
  {
    /* When the netif is fully configured this function must be called.*/
    netif_set_up(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif);
  }

  /* Set the link callback function, this function is called on change of link status*/
  netif_set_link_callback(&gnetif, ethernetif_update_config);
  
  /* create a binary semaphore used for informing ethernetif of frame reception */
  osSemaphoreDef(Netif_SEM);
  Netif_LinkSemaphore = osSemaphoreCreate(osSemaphore(Netif_SEM) , 1 );
  
  link_arg.netif = &gnetif;
  link_arg.semaphore = Netif_LinkSemaphore;
  /* Create the Ethernet link handler thread */
  osThreadDef(LinkThr, ethernetif_set_link, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadCreate (osThread(LinkThr), &link_arg);
}

/**
  * @brief  Initializes the LCD and LEDs resources.
  * @param  None
  * @retval None
  */
static void BSP_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
   
   __GPIOB_CLK_ENABLE();
   
  GPIO_InitStructure.Pin = GPIO_PIN_14;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL ;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0xF, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  
  /* Initialize LEDs */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
}

/**
  * @brief  Toggle LED4 task
  * @param  pvParameters not used
  * @retval None
  */
static void ToggleLed4(void const * argument)
{
  for( ;; )
  {
    /* toggle LED4 each 250ms */
    BSP_LED_Toggle(LED4);
    osDelay(250);
  }
}

/**
  * @brief  EXTI line detection callbacks
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_14)
  {
    osSemaphoreRelease(Netif_LinkSemaphore);
    
  }
}

/**
  * @brief  key task
  * @param  pvParameters not used
  * @retval None
  */
char *baidu={"www.baidu.com"};
struct ip_addr baidu_addr;
xQueueHandle xQueue;
void KeyTask(void * pvParameters)
{ 
  uint32_t key_value=0;
  uint32_t lValueToSend;
  portTickType xLastWakeTime;
  static portBASE_TYPE xStatus;
  //err_t err;
  
  lValueToSend = (uint16_t)SignInReq;//(uint32_t)pvParameters;
  xQueue = xQueueCreate(1,sizeof(uint16_t));
  while(1)
  {
    key_value = BSP_PB_GetState(BUTTON_WAKEUP);
    if(key_value)
    {
      //tcp_client_flag |= LWIP_SEND_DATA;
      xStatus = xQueueSendToBack(xQueue, &lValueToSend,0);
      if(xStatus != pdPASS)
      {
        xQueueSendToBack(xQueue, &lValueToSend,0);
        printf("Could not send to the queue.\r\n");
      }
      //err = dns_gethostbyname(baidu,&baidu_addr,my_found,NULL);
      //taskYIELD(); 任务等级低所以不需要，切换
    }
    vTaskDelayUntil(&xLastWakeTime,(250/portTICK_RATE_MS)); //frequce is 4hz(every one second four times)
  }     
}

static void StartThread1(void const * argument) {

  /* Infinite loop */
  for(;;)
  {
    osDelay(2);
  }

  /* USER CODE END 5 */ 

}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
/* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
