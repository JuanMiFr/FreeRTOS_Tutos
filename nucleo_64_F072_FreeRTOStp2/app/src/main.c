
#include "stm32f0xx.h"
#include "bsp.h"
#include "main.h"
#define N (4)
#define SENSOR_TABLE_SIZE 2



// Static functions
static void SystemClock_Config	(void);



// FreeRTOS tasks
void vTask1 (void *pvParameters);
void vTask2 (void *pvParameters);
void vTask_Pub (void *pvParameters);


/*
* Kernel objects
*/
xQueueHandle xSubscribeQueue;
xSemaphoreHandle xSem1;
xSemaphoreHandle xSem2;

typedef struct
{
uint8_t sem_id; // Semaphore ID to use for publication
uint8_t sensor_id; // Awaited sensor ID
uint8_t sensor_state; // Awaited sensor State
} subscribe_message_t;




// Main function
int main()
{
	// Configure System Clock
	SystemClock_Config();
	// Initialize LED pin
	BSP_LED_Init();
	// Initialize Debug Console
	BSP_PB_Init();
	BSP_Console_Init();
	my_printf("Console Ready!\r\n");
	//my_printf("SYSCLK=%d\r\n",Systick);


	// Start Trace Recording
	xTraceEnable(TRC_START);

    // Create Queue to hold console messages
	xSubscribeQueue = xQueueCreate(4, sizeof(subscribe_message_t));

	// Give a nice name to the Queue in the trace recorder
	vTraceSetQueueName(xSubscribeQueue, "subscriber");
	// Create Semaphore object (this is not a 'give')
	xSem1 = xSemaphoreCreateBinary();
	xSem2 = xSemaphoreCreateBinary();
	// Give a nice name to the Semaphore in the trace recorder
	//vTraceSetSemaphoreName(xSem, "xSEM");
	// Create Tasks

	xTaskCreate(vTask1, "Task_1", 256, NULL, 1, NULL);
	xTaskCreate(vTask2, "Task_2", 256, NULL, 2, NULL);
	xTaskCreate(vTask_Pub,"Task_Pub", 256 , NULL, 3, NULL);
	// Start the Scheduler
	vTaskStartScheduler();
	while(1)
	{
		// The program should never be here...
	}
}

/*
 *	Task_1 toggles LED every 10ms
 */
void vTask1 (void *pvParameters)
{
	subscribe_message_t suscribe_t1;

	while(1)
	{
		suscribe_t1.sem_id=1;
		suscribe_t1.sensor_id=1;
		suscribe_t1.sensor_state=1;
		xQueueSendToBack(xSubscribeQueue, &suscribe_t1, 0);
		xSemaphoreTake(xSem1,portMAX_DELAY);

		BSP_LED_On();
		BSP_DELAY_ms(100);
		BSP_LED_Off();

		suscribe_t1.sem_id=1;
		suscribe_t1.sensor_id=1;
		suscribe_t1.sensor_state=0;
		xQueueSendToBack(xSubscribeQueue, &suscribe_t1, 0);
		xSemaphoreTake(xSem1,portMAX_DELAY);
		// Two short flashes
		BSP_LED_On();
		BSP_DELAY_ms(100);
		BSP_LED_Off();
		BSP_DELAY_ms(200);
		BSP_LED_On();
		BSP_DELAY_ms(100);
		BSP_LED_Off();


		// Send message to the Console Queue
		//xQueueSendToBack(xSubscribeQueue, &suscribe_t1, 0);
		//vTaskDelay(5000);

	}
}
/*
 *	Task_2 sends a message to console when xSem semaphore is given
 */
void vTask2 (void *pvParameters)
{
	// Take the semaphore once to make sure it is empty
	//xSemaphoreTake(xSem, 0);
	subscribe_message_t suscribe_t2;

	while(1)
	{

		suscribe_t2.sem_id=2;
		suscribe_t2.sensor_id=2;
		suscribe_t2.sensor_state=1;
		xQueueSendToBack(xSubscribeQueue, &suscribe_t2, 0);
		xSemaphoreTake(xSem2,portMAX_DELAY);

		BSP_LED_On();
		BSP_DELAY_ms(500);
		BSP_LED_Off();

		suscribe_t2.sem_id=2;
		suscribe_t2.sensor_id=2;
		suscribe_t2.sensor_state=0;
		xQueueSendToBack(xSubscribeQueue, &suscribe_t2, 0);
		xSemaphoreTake(xSem2,portMAX_DELAY);

		// Two short flashes
		BSP_LED_On();
		BSP_DELAY_ms(500);
		BSP_LED_Off();
		BSP_DELAY_ms(200);
		BSP_LED_On();
		BSP_DELAY_ms(500);
		BSP_LED_Off();




		//xQueueSendToBack(xSubscribeQueue, &suscribe_t2, 0);
		//vTaskDelay(4000);
	}
}

void vTask_Pub (void *pvParameters)
{
	subscribe_message_t suscribe;
	//portBASE_TYPE   xStatus;

	// Initialize timing
	portTickType	xLastWakeTime;

	uint8_t sensor[SENSOR_TABLE_SIZE];
	sensor[0]=0;
	sensor[1]=0;
	xLastWakeTime = xTaskGetTickCount();
	subscribe_message_t tab[N]; //tableau des subscribers de taille N
	for(uint8_t k=0;k<N;k++)
	{
		tab[k].sem_id=0;
		tab[k].sensor_id=0;
		tab[k].sensor_state=0;
	}

	uint8_t New=1;
	while (1)
	{



		// Wait for something in the message Queue
		if (xQueueReceive(xSubscribeQueue, &suscribe,0))
		{
			uint8_t existe = 0;
			int8_t  free = -1;


			// Send message to console
			my_printf("\r\nSubscribing : SemID=%d SensID=%d State=%d\r\n",suscribe.sem_id,suscribe.sensor_id,suscribe.sensor_state);
			//my_printf("Je suis interessee par l’'etat %d du capteur %d, merci de me le signaler via le semaphore %d\r\n",suscribe.sem_id,suscribe.sensor_id,suscribe.sensor_state);

			for (uint8_t i=0;i<N;i++)
			{
				if(suscribe.sem_id==tab[i].sem_id)
				{
					my_printf("Subscription already exists\r\n");
					existe=1;
					//New=-1;

				}
				if( (tab[i].sem_id==0) && (free==-1))//(free = -1)
				{
					free = i;
				}
				//if( (tab[i].sem_id==0) && (free==-1))//(free = -1)
				//{
				//	my_printf("Adding subscription in slot [%d]\r\n",i);
				//	tab[i].sem_id=suscribe.sem_id;
				//	tab[i].sensor_id=suscribe.sensor_id;
				//	tab[i].sensor_state=suscribe.sensor_state;
				//	New=-1;
				//}
			}

			if (existe==0)
			{
				tab[free].sem_id=suscribe.sem_id;
				tab[free].sensor_id=suscribe.sensor_id;
				tab[free].sensor_state=suscribe.sensor_state;
				my_printf("Adding subscription in slot [%d]\r\n",free);
			}
			my_printf("[0] %d %d %d\r\n[1] %d %d %d\r\n[2] %d %d %d\r\n[3] %d %d %d\r\n",tab[0].sem_id,tab[0].sensor_id,tab[0].sensor_state,tab[1].sem_id,tab[1].sensor_id,tab[1].sensor_state,tab[2].sem_id,tab[2].sensor_id,tab[2].sensor_state,tab[3].sem_id,tab[3].sensor_id,tab[3].sensor_state);


		}else
		{
			my_printf(".");
		}

		vTaskDelayUntil(&xLastWakeTime, 200);

		//New=1;
		uint8_t rx_byte=0;
		// If there is something in USART RDR register
		if ( (USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE )
		{
			// Read and report the content of RDR
			rx_byte = USART2->RDR;
			switch (rx_byte)
			{
				case 97://a
					sensor[0]=0;
					break;
				case 98://b
					sensor[0]=1;
					break;
				case 99://c
					sensor[1]=0;
					break;
				case 100: //d
					sensor[1]=1;
					break;
				default:
					break;
			}

		my_printf("\r\nSensor state = [%d  %d] \r\n",sensor[0],sensor[1]);
		}


		for (uint8_t i=0; i<N;i++)
		{
			if(tab[i].sem_id!=0)
			{
				if(tab[i].sensor_state==sensor[tab[i].sensor_id-1])
				{
					if(tab[i].sem_id==1)
					{
						xSemaphoreGive(xSem1);
						my_printf("Sem1");
					}else if (tab[i].sem_id==2)
					{
						xSemaphoreGive(xSem2);
						my_printf("Sem2");
					}
					tab[i].sem_id=0;
					tab[i].sensor_id=0;
					tab[i].sensor_state=0;

				}
			}
		}
		// Wait here for 200ms since last wakeup
		//vTaskDelayUntil (&xLastWakeTime, (200/portTICK_RATE_MS));
	}
}






/*
 * 	Clock configuration for the Nucleo STM32F072RB board
 * 	HSE input Bypass Mode 			-> 8MHz
 * 	SYSCLK, AHB, APB1 			-> 48MHz
 *  	PA8 as MCO with /16 prescaler 		-> 3MHz
 *
 *  Laurent Latorre - 05/08/2017
 */

static void SystemClock_Config()
{
	uint32_t	HSE_Status;
	uint32_t	PLL_Status;
	uint32_t	SW_Status;
	uint32_t	timeout = 0;
	timeout = 1000000;
	// Start HSE in Bypass Mode
	RCC->CR |= RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON;
	// Wait until HSE is ready
	do
	{
		HSE_Status = RCC->CR & RCC_CR_HSERDY_Msk;
		timeout--;
	} while ((HSE_Status == 0) && (timeout > 0));
	// Select HSE as PLL input source
	RCC->CFGR &= ~RCC_CFGR_PLLSRC_Msk;
	RCC->CFGR |= (0x02 <<RCC_CFGR_PLLSRC_Pos);
	// Set PLL PREDIV to /1
	RCC->CFGR2 = 0x00000000;
	// Set PLL MUL to x6
	RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk;
	RCC->CFGR |= (0x04 <<RCC_CFGR_PLLMUL_Pos);
	// Enable the main PLL
	RCC-> CR |= RCC_CR_PLLON;
	// Wait until PLL is ready
	do
	{
		PLL_Status = RCC->CR & RCC_CR_PLLRDY_Msk;
		timeout--;
	} while ((PLL_Status == 0) && (timeout > 0));
        // Set AHB prescaler to /1
	RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	//Set APB1 prescaler to /1
	RCC->CFGR &= ~RCC_CFGR_PPRE_Msk;
	RCC->CFGR |= RCC_CFGR_PPRE_DIV1;
	// Enable FLASH Prefetch Buffer and set Flash Latency
	FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
	/* --- Until this point, MCU was still clocked by HSI at 8MHz ---*/
	/* --- Switching to PLL at 48MHz Now!  Fasten your seat belt! ---*/
	// Select the main PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	// Wait until PLL becomes main switch input
	do
	{
		SW_Status = (RCC->CFGR & RCC_CFGR_SWS_Msk);
		timeout--;
	} while ((SW_Status != RCC_CFGR_SWS_PLL) && (timeout > 0));
	/* --- Here we go! ---*/
	/*--- Use PA8 as MCO output at 48/16 = 3MHz ---*/
	// Set MCO source as SYSCLK (48MHz)
	RCC->CFGR &= ~RCC_CFGR_MCO_Msk;
	RCC->CFGR |=  RCC_CFGR_MCOSEL_SYSCLK;
	// Set MCO prescaler to /16 -> 3MHz
	RCC->CFGR &= ~RCC_CFGR_MCOPRE_Msk;
	RCC->CFGR |=  RCC_CFGR_MCOPRE_DIV16;
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	// Configure PA8 as Alternate function
	GPIOA->MODER &= ~GPIO_MODER_MODER8_Msk;
	GPIOA->MODER |= (0x02 <<GPIO_MODER_MODER8_Pos);
	// Set to AF0 (MCO output)
	GPIOA->AFR[1] &= ~(0x0000000F);
	GPIOA->AFR[1] |=  (0x00000000);
	// Update SystemCoreClock global variable
	SystemCoreClockUpdate();
}

