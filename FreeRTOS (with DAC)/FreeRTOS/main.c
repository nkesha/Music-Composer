/* This FreeRTOS STM32F4 based on:
https://www.instructables.com/Setting-Up-FreeRTOS-From-Scratch-on-STM32F407-Disc/ 
Remarks:
This template is upgraded to the latest FreeRTOS  202012.00 (10.4.3)

FreeRTOSConfig has been added into \FreeRTOS_STM32F4\FreeRTOSv202012.00\FreeRTOS\Source\portable\RVDS\ARM_CM4F

heap_4.c is included in the project (STM32f407VG board) to provide the memory allocation required by the RTOS kernel

System clock is based on HSE configuration

PendSV, SysTcik and SVC handlers "define" have been added to port.c
#define  xPortPendSVHandler      PendSV_Handler 
#define  xPortSysTickHandler     SysTick_Handler
#define  vPortSVCHandler         SVC_Handler
*/

/* FreeRTOS includes. */
#include <stdio.h>
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "stdbool.h"
#include "MY_CS43L22.h"

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/
#define PI 3.14159f
//Sample rate and Output freq
#define F_SAMPLE		48000.0f // Sample freq
#define F_OUT				880.0f  // Output freq 
#define AMP				1000  // Output amplitute 
  
#define A5				880.0f  
#define B5				988.0f
#define C6				1047.0f
#define D6				1175.0f  
#define E6				1319.0f   
#define F6				1397.0f   
#define G6				1568.0f 
#define A6				1760.0f 

#define numTones  8
#define numLengths  4
#define set_value(i, value) (*((&(TIM4->CCR1)) + i) = value) //macro function to update the channel value at timer 4
#define TONE_MAX 8
#define PWM_MAX 5000


float listTones[numTones] = {A5, B5, C6, D6, E6, F6, G6, A6}; //Array contaning all the tones frequency available
float servosPositions[numTones] = {500,800,1100,1400,1700,2000,2300,2600}; //Array containing all the servos postions available corresponding to available tones
int listLengths[numLengths] = {500,1000, 1500, 2000};   // Array corresponding to the lenghts of tones available
uint16_t arrayLed[numLengths] = {GPIO_PIN_13,GPIO_PIN_14,GPIO_PIN_15,GPIO_PIN_12}; //LEDS showing each length is selecting/playing

// Array containing choosen tones and lengths by the user
float listTonesToPlay[TONE_MAX];
int listLengthsToPlay[TONE_MAX];


typedef enum {selectTone, selectLength, playMusic} composerState; //State to represent what state the system is currently in

/*
	Button state: to show the number of presses the user is pressing
*/
typedef struct
{
	int numPresses;
	
} button;
/*
	Music Composer structure
*/
typedef struct
{
	int tone;  //postion of the current tone in our lists of tones
	int length; ////position of the current length in our lists of lengths
	composerState currState; //The curent state 
	int numTonesToPlay; //The current tones and length chosen by the user
} composer;



/* The task functions prototype*/
void vTaskBlueLED( void *pvParameters );
void vTaskOrangeLED( void *pvParameters );
void vTaskButton( void *pvParameters );
void vTaskBeep( void *pvParameters ) ;
void vTaskMoveServos( void *pvParameters );
void vTaskSelectLength (void *pvParameters);
void vTaskPlayMusic( void *pvParameters );

/* Global Variables */
GPIO_InitTypeDef GPIO_InitStruct; // Initiate the structure that contains the configuration information for the specified GPIO peripheral.
TIM_HandleTypeDef TIM_InitStruct;  // Initiate the structure that contains the configuration information for the timer

GPIO_InitTypeDef GPIO_InitStruct_PWM; // Initiate the structure that contains the configuration information for the specified GPIO peripheral.
TIM_HandleTypeDef TIM4_InitStruct;
TIM_HandleTypeDef TIM3_InitStruct;
TIM_OC_InitTypeDef TIM4_OCInitStructure; // Initiate the structure that contains the configuration information for the specified output capture
TIM_OC_InitTypeDef TIM3_OCInitStructure;
I2C_HandleTypeDef hi2c1;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;


// Tasks handlers
TaskHandle_t xBlueLEDHandler = NULL;
TaskHandle_t xButtonHandler = NULL;
TaskHandle_t xSoundHandler = NULL;
TaskHandle_t xMoveServosHandler = NULL;
TaskHandle_t xLengthHandler = NULL;
TaskHandle_t xPlayMusicHandler = NULL;

//Timers Handlers
TimerHandle_t buttonPressTimer;

float mySinVal;
float sample_dt;
uint16_t sample_N;
uint16_t i_t;
uint32_t myDacVal;
int delay;

int16_t dataI2S[100];
button userButton;
composer musicComposer;

void initComposer()
{
	
	musicComposer.tone = 0;
	musicComposer.length = 0;
	musicComposer.currState = selectTone;
	musicComposer.numTonesToPlay = 0;
	// Position the servos to the first state
	TIM3->CCR1 = servosPositions[musicComposer.tone];
	TIM3->CCR2 = servosPositions[musicComposer.tone];
	
}

void SysTick_Handler(void)
{
  HAL_IncTick();
	
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
			xPortSysTickHandler();
  }
	
}

void EXTI0_IRQHandler(void) 
{
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
	

	// Resume the button task from the ISR
	BaseType_t xYieldRequiredCheck = xTaskResumeFromISR(xButtonHandler);
	portYIELD_FROM_ISR(xYieldRequiredCheck); // Yield to avoid delayed until the next time the scheduler

}

void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
}

void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
	
  // Enable Power Control clock
  __PWR_CLK_ENABLE();

  // The voltage scaling allows optimizing the power consumption when the
  // device is clocked below the maximum system frequency, to update the
  // voltage scaling value regarding system frequency refer to product
  // datasheet.
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Enable HSE Oscillator and activate PLL with HSE as source
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  // Based on the STM32F407 HSE clock configuration
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  // Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  // clocks dividers
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
	
	// Select peripheral clock for I2S
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 100;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
}

void InitLED(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();  // Enable clock to GPIO-D for LEDs
    // Set GPIOD Pins Parameters 
    GPIO_InitStruct.Pin     = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;  // I/O PD12, PD13, PD14, and PD15
    GPIO_InitStruct.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull    = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);    // Init GPIOD 
}

void InitGPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  //__HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level LEDs and DAC output */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void InitButton(void)
{
	  __HAL_RCC_GPIOA_CLK_ENABLE();  // Enable clock to GPIO-A for button
    // Set GPIOA Pin Parameters, pin 0 for button
    GPIO_InitStruct.Pin     = GPIO_PIN_0;
	  // Specifies the trigger signal active edge for the EXTI lines. (e.g. Rising = button is pressed)
    GPIO_InitStruct.Mode    = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull    = GPIO_NOPULL;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  // Init GPIOA
		HAL_NVIC_SetPriority(EXTI0_IRQn, 10, 0);
		HAL_NVIC_EnableIRQ(EXTI0_IRQn); // Enable GPIO_PIN_0 interrupt at IRQ-Level
}	

void InitI2C1(void)
{
		hi2c1.Instance = I2C1;
		hi2c1.Init.ClockSpeed = 100000; // clock frequency <= 400kHz
		hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2; //fast mode Tlow/Thigh = 2
		hi2c1.Init.OwnAddress1 = 0; 
		hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		hi2c1.Init.OwnAddress2 = 0; 
		hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		HAL_I2C_Init(&hi2c1); //initialize the I2C peripheral 
}

void InitI2S3(void)
{
		hi2s3.Instance = SPI3;
		hi2s3.Init.Mode = I2S_MODE_MASTER_TX; //transmit in master mode
		hi2s3.Init.Standard = I2S_STANDARD_PHILIPS; //data protocol
		hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B; //bits per sample
		hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE; //master clock signal
		hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K; //audio frequency
		hi2s3.Init.CPOL = I2S_CPOL_LOW; //clock polarity
		hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
		hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
		HAL_I2S_Init(&hi2s3); //set the setting 
}
void InitDMA(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}


void InitServo(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();  // Enable clock to GPIO-B for PB6 and PB7
    // Set GPIOB Pins Parameters, PB6 and PB7 
		GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;  //PB6: TIM4_CH1 and PB7: TIM4_CH2
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; //Alternate Function Push Pull Mode 
    GPIO_InitStruct.Pull = GPIO_NOPULL; // No Pull-up or Pull-down activation
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3; // Assign those pins alternate function in TIM4 
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // Init GPIOB
		
}


void InitTimer3(void)
{
		//__TIM1_CLK_ENABLE();
		__HAL_RCC_TIM3_CLK_ENABLE(); // Enable clock to TIM3 from APB2 bus (48Mhz max)xPLL_P = 84MHz
		 
	// TIM4 is configure to 50hz: 50 times in 1s or 1000000us
    TIM3_InitStruct.Instance = TIM3;
		TIM3_InitStruct.Init.Period = 20000-1;
    TIM3_InitStruct.Init.Prescaler   = 84-1;
		TIM3_InitStruct.Init.CounterMode = TIM_COUNTERMODE_UP;
		TIM3_InitStruct.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		TIM3_InitStruct.Init.RepetitionCounter = 0;
		HAL_TIM_Base_Init(&TIM3_InitStruct); // Init TIM3
		
		/*//if you would like to enable interrupt
		//HAL_TIM_Base_Start_IT(&TIM3_InitStruct); // Enable timer-3 IRQ interrupt
		//HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
		//HAL_NVIC_EnableIRQ(TIM3_IRQn); // Enable interrupt at IRQ-Level
		*/
    HAL_TIM_Base_Start(&TIM3_InitStruct); // Start TIM3
}



void SetupPWM_TIM3()
{
	HAL_TIM_PWM_Init(&TIM3_InitStruct);
	
	TIM3_OCInitStructure.OCMode = TIM_OCMODE_PWM1; //Set output capture as PWM mode
  TIM3_OCInitStructure.Pulse = 0; // Initial duty cycle at 0%
  TIM3_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH; // HIGH output compare active
  TIM3_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE; // output compare disable
	HAL_TIM_PWM_ConfigChannel(&TIM3_InitStruct, &TIM3_OCInitStructure, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TIM3_InitStruct, TIM_CHANNEL_1); // Start PWM at channel 1
	HAL_TIM_PWM_ConfigChannel(&TIM3_InitStruct, &TIM3_OCInitStructure, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&TIM3_InitStruct, TIM_CHANNEL_2); // Start PWM at channel 2
}



int SCHED_TYPE = 0;
void buttonPressTimerCallback(TimerHandle_t buttonPressTimer)
{
	printf("Number Clicks value = %d \n", userButton.numPresses);

	// Single Press Button
	if (userButton.numPresses == 1)
	{

		HAL_GPIO_WritePin(GPIOD, arrayLed[SCHED_TYPE], GPIO_PIN_SET);
		SCHED_TYPE++;
		if(SCHED_TYPE == 4)
			SCHED_TYPE = 0;
		
	// Double Press
	}
	else if(userButton.numPresses == 2)
	{
		
	
	}


}


// Turns all LEDs off
void allLedsOff(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
}

// Turns all LEDs on
void allLedsOn(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
}

int main( void )
{

	SystemInit();
	HAL_Init();
  SystemClock_Config();
	InitButton();
	InitGPIO();
	InitLED();
	InitServo();
	InitTimer3();
	SetupPWM_TIM3();
	
	InitDMA();
	InitI2C1();
	InitI2S3();
	initComposer();

	
	

	//Tasks

	xTaskCreate( vTaskButton, "vTaskButton", STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, &xButtonHandler );
	xTaskCreate( vTaskMoveServos, "vTaskServos", STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, &xMoveServosHandler );
	xTaskCreate( vTaskSelectLength, "vTaskSelectLength", STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, &xLengthHandler );
	xTaskCreate( vTaskPlayMusic, "vTaskPlayMusic", STACK_SIZE_MIN*4, NULL, 2, &xPlayMusicHandler);
	
	//Timers
	buttonPressTimer = xTimerCreate("buttonPressTimer", 1500/portTICK_PERIOD_MS, pdFALSE, (void*)0, buttonPressTimerCallback);
	
	vTaskStartScheduler();


	for( ;; );
}



void vTaskButton( void *pvParameters ) // Button Task (interrupt)
{
	const char *pcTaskName = "vTaskButton is running\n";
	
	for(;;)
	{
		vTaskSuspend(NULL);
		printf("%s\n",pcTaskName);
		if (userButton.numPresses == 0)
		{
			xTimerStart(buttonPressTimer, 0);
		}
		userButton.numPresses++;
		vTaskDelay(200);
	}
}
void vTaskMoveServos( void *pvParameters ) // Moving servos to next position
{
	const char *pcTaskName = "vTaskMoveServos is running\n";
	
	for(;;)
	{
		vTaskSuspend(NULL);

		printf("%s\n",pcTaskName);
		
		//Go next postion servos
		musicComposer.tone++;
		if(musicComposer.tone >= numTones)
			musicComposer.tone = 0;
		TIM3->CCR1 = servosPositions[musicComposer.tone];
		TIM3->CCR2 = servosPositions[musicComposer.tone];	
		
		//Resetting the number of presses to zero
		userButton.numPresses = 0;
	}
		
}



void vTaskSelectLength (void *pvParameters)
{
	const char *pcTaskName = "vTaskSelectLength is running\n";
	
	for(;;)
	{
		vTaskSuspend(NULL);
		printf("%s\n",pcTaskName);
		allLedsOff();

		if(musicComposer.currState == selectTone && userButton.numPresses == 2)
		{
			
			musicComposer.currState = selectLength;
			listTonesToPlay[musicComposer.numTonesToPlay] = listTones[musicComposer.tone];
			printf("Tone that will play %d \n",(int) listTonesToPlay[musicComposer.numTonesToPlay]);
      HAL_GPIO_WritePin(GPIOD, arrayLed[musicComposer.length], GPIO_PIN_SET);

		}
		else if (musicComposer.currState == selectLength && userButton.numPresses == 1)
		{
			musicComposer.length++;
			if(musicComposer.length >= numLengths)
				musicComposer.length = 0;
			HAL_GPIO_WritePin(GPIOD, arrayLed[musicComposer.length], GPIO_PIN_SET);
		}
		//Select the length to play
		else if (musicComposer.currState == selectLength && userButton.numPresses == 2)
		{
			//Save the current length to be played later
			listLengthsToPlay[musicComposer.numTonesToPlay] = listLengths[musicComposer.length];
			printf("Length that will play %d \n",listLengthsToPlay[musicComposer.numTonesToPlay]);
			printf("Number of Tones To Play %d \n",musicComposer.numTonesToPlay);
			musicComposer.numTonesToPlay++;
			//case When we have already 8 tones and 8 lengths
			if(musicComposer.numTonesToPlay == TONE_MAX)
			{
				allLedsOn();
				musicComposer.currState = playMusic;
			}
			// Go Back to selection Tone Mode
			else
			{
				
				HAL_GPIO_WritePin(GPIOD, arrayLed[musicComposer.length],GPIO_PIN_RESET);				
				musicComposer.currState = selectTone; //Return to selectTone Mode
				musicComposer.length = 0; 	//Reset the length pointer back to first length in our list

			}			
		}
		//Resetting the number of presses to zero
		userButton.numPresses = 0;
	}
		
}


//This function will play a given tone provided by the user
void vTaskPlayMusic( void *pvParameters )
{
	const char *pcTaskName = "vTaskPlayMusic is running\n";
	vTaskSuspend(NULL);
	int i = 0;
	int stop = musicComposer.numTonesToPlay;
	printf("\nTotal Tones To play %d\n",stop);
	for(;;)
	{
		printf("%s\n",pcTaskName);

		if(i == stop)
		{
				allLedsOn();
				vTaskSuspend(NULL);
		}
		else
		{
			CS43_Init(hi2c1, MODE_I2S);
			CS43_SetVolume(50); //0 - 100
			CS43_Enable_RightLeft(CS43_RIGHT_LEFT);
			CS43_Start();
			sample_dt = listTonesToPlay[i]/F_SAMPLE;
			printf("Tone playing at list tone selected position %d \n", i);
			printf("Frequency playing %d \n", (int) listTonesToPlay[i]);
			delay = listLengthsToPlay[i];
			printf("Delay %d \n", listLengthsToPlay[i]);
			sample_N =  F_SAMPLE/listTonesToPlay[i];
			printf(" sample_N = %d\n", sample_N);
			for(uint16_t j=0; j<sample_N; j++)
			{
				mySinVal = AMP*sinf(j*2*PI*sample_dt); 
				dataI2S[j*2] = mySinVal;    //Right data (even: 0,2,4,6...)
				dataI2S[j*2 + 1] = mySinVal; //Left data  (odd: 1,3,5,7...)
				printf(" j = %d\n", j);
			}
			HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)dataI2S, sample_N*2);
			vTaskDelay(delay);
			HAL_I2S_DMAStop(&hi2s3);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4,GPIO_PIN_RESET);
			CS43_Stop();
			
			
			printf("%d\n",i);
			printf("stop %d\n",stop);
			i++;
			
		}

	}
}
void vApplicationMallocFailedHook( void )
{
	/* This function will only be called if an API call to create a task, queue
	or semaphore fails because there is too little heap RAM remaining. */
	for( ;; );
}

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{

	for( ;; );
}


void vApplicationIdleHook( void )
{
	/* This example does not use the idle hook to perform any processing. */
	//HAL_IncTick();
	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
}

void vApplicationTickHook( void )
{

}
