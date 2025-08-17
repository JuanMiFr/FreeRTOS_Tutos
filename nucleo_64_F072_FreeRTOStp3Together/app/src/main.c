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
void vTask_Write (void *pvParameters);


/*
* Kernel objects
*/
xQueueHandle xWriteQueue;
xSemaphoreHandle xSem_UART_TC;
xSemaphoreHandle xSem_DMA_TC;
//xSemaphoreHandle xSem2;

//typedef struct
// Define the command_message_t type as an array of xx char
typedef uint8_t command_message_t[48];
uint8_t tx_dma_buffer[130];
//{
//uint8_t sem_id; // Semaphore ID to use for publication
//uint8_t sensor_id; // Awaited sensor ID
//uint8_t sensor_state; // Awaited sensor State
//} subscribe_message_t;




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
	//my_printf("Console Ready!\r\n");
	//my_printf("SYSCLK=%d\r\n",Systick);

	// Start Trace Recording
	xTraceEnable(TRC_START);


    // Create Queue to hold console messages
	xWriteQueue = xQueueCreate(4, sizeof(command_message_t));

	// Give a nice name to the Queue in the trace recorder
	vTraceSetQueueName(xWriteQueue, "writer");
	// Create Semaphore object (this is not a 'give')
	xSem_UART_TC = xSemaphoreCreateBinary();
	xSem_DMA_TC = xSemaphoreCreateBinary();
	// Give a nice name to the Semaphore in the trace recorder
	vTraceSetSemaphoreName(xSem_UART_TC, "xSEM_UART_TC");
	vTraceSetSemaphoreName(xSem_DMA_TC, "xSEM_DMA_TC");
	//xSem2 = xSemaphoreCreateBinary();

	// Create Tasks
	xTaskCreate(vTask1, "Task_1", 128, NULL, 1, NULL);
	xTaskCreate(vTask2, "Task_2", 128, NULL, 2, NULL);
	xTaskCreate(vTask_Write,"Task_Write", 128 , NULL, 3, NULL);
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
	command_message_t write_t1;

	while(1)
	{
		// Prepare message
		my_sprintf((char *)write_t1, "Arthour ! Pas changer assiette pour fromage !\r\n");
		// Send message to the Console Queue
		xQueueSendToBack(xWriteQueue, &write_t1, 0);
		//xSemaphoreTake(xSem1,portMAX_DELAY);
		vTaskDelay(100);

	}
}
/*
 *	Task_2 sends a message to console when xSem semaphore is given
 */
void vTask2 (void *pvParameters)
{
	// Take the semaphore once to make sure it is empty
	//xSemaphoreTake(xSem, 0);
	command_message_t write_t2;

	while(1)
	{
		// Prepare message
		my_sprintf((char *)write_t2, "Arthour ! Couhillere !\r\n");
		// Send message to the Console Queue
		xQueueSendToBack(xWriteQueue, &write_t2, 0);
		//xSemaphoreTake(xSem1,portMAX_DELAY);
		vTaskDelay(100);
	}
}

/*
 *	Task_write sends a message to console when xSem semaphore is given
 */
void vTask_Write (void *pvParameters)
{
	command_message_t msg;
	uint32_t n = 0;  // Compteur pour les octets dans le message
	uint8_t *p_msg = NULL;
	// Set priority level 1 for USART2 interrupt
	NVIC_SetPriority(USART2_IRQn, 6);
	// Enable USART2 interrupts
	NVIC_EnableIRQ(USART2_IRQn);



	NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, configMAX_API_CALL_INTERRUPT_PRIORITY + 1);
	NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

	while(1)
	{
		// Attente du message dans la file d'attente
		xQueueReceive(xWriteQueue, &msg, portMAX_DELAY);

		// Pointeur vers le message reçu
		p_msg = (uint8_t *)msg;

		// Copier le message dans le buffer tx_dma_buffer et compter les octets
		n = 0;
		while (p_msg[n] != '\0' && n < sizeof(tx_dma_buffer))  // On s'arrête au '\0' ou à la taille du buffer
		{
			tx_dma_buffer[n] = p_msg[n];  	// Copie dans le buffer DMA
			n++;  							// Incrémenter le compteur d'octets
		}

		// Configurer la taille du transfert DMA (CNDTR)
		DMA1_Channel4->CNDTR = n;


		//my_printf("a");

		// Activer le canal DMA pour démarrer la transmission
		DMA1_Channel4->CCR |= DMA_CCR_EN;
		// Préparer l'USART2 pour la transmission par DMA
		USART2->CR3 |= USART_CR3_DMAT;  // Activer la requête DMA pour TX

		// Attendre que la transmission DMA soit terminée (sémaphore)
		xSemaphoreTake(xSem_DMA_TC, portMAX_DELAY);

		// Désactiver le DMA et la requête DMA de l'USART2 après la transmission
		DMA1_Channel4->CCR &= ~DMA_CCR_EN;
		USART2->CR3 &= ~USART_CR3_DMAT;

		//my_printf("b");

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
