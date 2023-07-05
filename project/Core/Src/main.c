/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "liquidcrystal_i2c.h"
#include "freertos.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f1xx_hal_tim_ex.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TIMER_CLOCK_FREQ 72000000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

SemaphoreHandle_t sem_UART1;
SemaphoreHandle_t Mutex;
SemaphoreHandle_t sem_Dmfd;

// Handler de la cola
QueueHandle_t queueUART1;
QueueHandle_t queuePWM;
QueueHandle_t queueLCD;
QueueHandle_t queueMFD;
QueueHandle_t queueDmfd;
QueueHandle_t queueSD;
QueueHandle_t queueDSD;
// datos de la colas

typedef struct {
	uint8_t code;
	uint8_t lfrequency[5];
	uint8_t ldutyCycle[5];
	uint8_t fecha[23];
} queueLCD_t;

typedef struct {
	float frequency;
	float dutyCycle;
} queueMFD_t;

uint8_t dataUART[23];
uint8_t dataPWM[8];
char cadena;
uint8_t queueS_1;
char cadenaSD_F[125];

// Handlers de tareas
TaskHandle_t h_UART1;
TaskHandle_t h_pwm;
TaskHandle_t h_mfd;
TaskHandle_t h_lcd;
TaskHandle_t h_Dmfd;
TaskHandle_t h_SD;
TaskHandle_t h_dsd;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;
////////////////////////////////////////////////////////////////////////////////
//// interrupcion TIMER 2 medicion de freq & duty
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

	float freq = 0;
	float duty = 0;
	queueMFD_t buffer = { 0 };

	if (htim->Instance == TIM2) {

		uint32_t cl = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		uint32_t ch = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

		freq = (float) TIMER_CLOCK_FREQ / (cl + 1);
		duty = (float) 100 * ch / cl;

		buffer.frequency = freq;

		buffer.dutyCycle = duty;

		xQueueSendToBackFromISR(queueMFD, &buffer, portMAX_DELAY);

		HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1); // Primary channel - rising edge
		HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_2);

	}
}

////////////////////////////////////////////////////////////////////////////////
//// interrupcion de uart
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	int indice;
	char buffer[23];
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
	if (huart->Instance == USART1) {

		buffer[indice++] = cadena;

		if (cadena == '&')
			indice = 0;

		if (indice >= 23)

		{

			indice = 0;
			xQueueSendToBackFromISR(queueUART1, &buffer, portMAX_DELAY);

			static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(sem_UART1, &xHigherPriorityTaskWoken);
			if (xHigherPriorityTaskWoken) {
				taskYIELD();
			}
		}
	}
	HAL_UART_Receive_IT(&huart1, (uint8_t*) &cadena, 1);

}

////////////////////////////////////////////////////////////////////////////////////
/// tarea presentacion

static void LCDTask(void *parameters) {

	queueLCD_t buffer_lcd = { 0 };
	char fecha[14];
	char frequency[5];
	char dutyCycle[2];

	while (1) {

		xQueueReceive(queueLCD, &buffer_lcd, portMAX_DELAY);

		if (buffer_lcd.code == 2) {

			HD44780_SetCursor(8, 0);
			HD44780_PrintStr((char*) buffer_lcd.lfrequency);
			HD44780_SetCursor(13, 0);
			HD44780_PrintStr(" Hz ");
			HD44780_SetCursor(8, 1);
			HD44780_PrintStr((char*) buffer_lcd.ldutyCycle);
			HD44780_SetCursor(13, 1);
			HD44780_PrintStr(" % ");

		}

		if (buffer_lcd.code == 1) {

			strncpy(fecha, (char*) buffer_lcd.fecha + 9, 23);
			HD44780_Clear();
			HD44780_SetCursor(1, 0);
			HD44780_PrintStr(fecha);
			HD44780_SetCursor(15, 0);
			HD44780_PrintStr(" ");
			HAL_Delay(3000);
			HD44780_Clear();
			strncpy(dutyCycle, (char*) buffer_lcd.fecha + 6, 8);
			dutyCycle[2] = '\0';
			strncpy(frequency, (char*) buffer_lcd.fecha, 5);
			frequency[5] = '\0';
			HD44780_SetCursor(0, 0);
			HD44780_PrintStr("(");
			HD44780_PrintStr(frequency);
			HD44780_PrintStr(")");
			HD44780_SetCursor(0, 1);
			HD44780_PrintStr("( ");
			HD44780_PrintStr(dutyCycle);
			HD44780_PrintStr(" %)");
		}

		vTaskDelay(500 / portTICK_PERIOD_MS);

	}

}

////////////////////////////////////////////////////////////////////////////////////
/// tarea Medicion

static void MFDTask(void *parameters) {
	float freq = 0;
	float duty = 0;
	uint32_t iduty = 0;
	uint32_t dduty = 0;
	uint32_t ifreq = 0;

	queueMFD_t buffer = { 0 };
	queueLCD_t buffer_lcd;

	while (1) {

		xQueueReceive(queueMFD, &buffer, portMAX_DELAY);

		freq = buffer.frequency;
		duty = buffer.dutyCycle;
		ifreq = (int) freq;
		iduty = (int) duty;
		dduty = (int) ((duty - iduty) * 1000);
		sprintf((char*) buffer_lcd.lfrequency, "%lu    ", ifreq);
		snprintf((char*) buffer_lcd.ldutyCycle, 19, "%2d.%2d ", iduty, dduty);
		buffer_lcd.code = 2;

		xQueueSendToBack(queueLCD, &buffer_lcd, portMAX_DELAY);

	}
}
////////////////////////////////////////////////////////////////////////////////////
/// tarea PWM

static void PWMTask(void *pvparameters)

{
	char datocolap[8];
	char frequencia[4];
	char dutycycle[2];

	while (1) {

		xQueueReceive(queuePWM, &datocolap, portMAX_DELAY);

//		HAL_UART_Transmit(&huart1, datocolap, sizeof(datocolap),100);

		strncpy(dutycycle, datocolap + 6, 8);
		dutycycle[2] = '\0';
		strncpy(frequencia, datocolap, 4);
		frequencia[5] = '\0';

		uint32_t freq = (uint32_t) strtol(frequencia, NULL, 10);
		uint32_t duty = (uint32_t) strtol(dutycycle, NULL, 10);

		// ajuste de pwm duty & freq

		uint32_t psc = 0, auxfreq = 0, uCclk = 0;
		psc = htim1.Init.Prescaler;
		uCclk = 72000000; // 72MHz
		auxfreq = uCclk / ((psc + 1) * freq);
		TIM1->ARR = auxfreq - 1;

		uint32_t dutyReg = 0, arr = 0;
		arr = TIM1->ARR;
		dutyReg = (arr * duty) / 100;
		TIM1->CCR1 = dutyReg;
		///////////////////////////////////////////

		vTaskDelay(1500 / portTICK_PERIOD_MS);

		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); // Primary channel - rising edge
		HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2); // Secondary channel - falling edge

	}
}

////////////////////////////////////////////////////////////////////////////////////
/// tarea disparo de medicion

static void DMFDTask(void *parameters) {

	while (1) {
		/*

		 HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); // Primary channel - rising edge
		 HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2); // Secondary channel - falling edge


		 vTaskDelay(5000 / portTICK_PERIOD_MS);

		 */

	}

}

////////////////////////////////////////////////////////////////////////////////////
///   datos de interrupcion UART

static void ReadUART1Task(void *pvParameters) {

	uint8_t state[23];
	char stateSD[23];
	char datocolap[8];
	queueLCD_t buffer_lcd = { 0 };

	while (1) {
		xSemaphoreTake(sem_UART1, portMAX_DELAY);
		//HAL_UART_Transmit(&huart1, state, sizeof(state),100);
		xQueueReceive(queueUART1, &state, portMAX_DELAY);

//		HAL_UART_Transmit(&huart1, state, sizeof(state),100);

		strcpy((char*) buffer_lcd.fecha, (char*) state);
		buffer_lcd.code = 1;

		xQueueSendToBack(queueLCD, &buffer_lcd, portMAX_DELAY);

		strncpy(datocolap, (char*) state, 8);
		datocolap[8] = '\0';
//		HAL_UART_Transmit(&huart1, state, sizeof(state),100);

		xQueueSendToBack(queuePWM, &datocolap, portMAX_DELAY);
		vTaskDelay(1200 / portTICK_PERIOD_MS);

		//HAL_UART_Transmit(&huart1, state, sizeof(state),100);

		strncpy(stateSD, (char*) state, 23);

		xQueueSendToBack(queueDSD, &state, portMAX_DELAY); //Enviar entero
		//vTaskDelay(1200 / portTICK_PERIOD_MS);

	}
}
/////////////////////////////////////////////////////////////////////////////
///Tarea SD

static void SDTask(void *pvParameters) {
	char data[120];

	portBASE_TYPE result = pdFALSE;
	while (1) {
		result = xQueueReceive(queueSD, &data, portMAX_DELAY);

//		taskENTER_CRITICAL();
		//HAL_UART_Transmit(&huart1, "SD", sizeof(char[2]),100);
		if (result) {

			//	strncpy(cadenaSD + indx, data, 23);
			//	cadenaSD[23] = '\n';

			//HAL_UART_Transmit(&huart1, data, sizeof(data),100);
			fresult = f_mount(&fs, "/", 1);
			/* Check free space */
			f_getfree("", &fre_clust, &pfs);
			total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
			free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
			/* Open file to write/ create a file if it doesn't exist */
			fresult = f_open(&fil, "pruebaD.txt",
					FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

			//HAL_UART_Transmit(&huart1, cadena, sizeof(data),100);
			/* Writing text */
			///f_puts(cadenaSD,&fil);
			f_puts(data, &fil);

			//f_write(&fil, cadenaSD, sizeof(cadenaSD), 200);
			/* Close file */
			fresult = f_close(&fil);

			//	HAL_UART_Transmit(&huart1, "fin", sizeof(char[3]),100);
			//////////////////////////////////////////////////////////

		}
//		taskEXIT_CRITICAL();
		//indx= indx+24;
		//vTaskDelete(h_SD);
	}
}

///////////////////////////////////////////////////////////////////////////////////
///Data SD
static void DSDTask(void *parameters) {
	char ty[120];
	char data[24];
	char cadenaSD[120] =
			"00000 00 00/00/00 00:00\n00000 00 00/00/00 00:00\n00000 00 00/00/00 00:00\n00000 00 00/00/00 00:00\n00000 00 00/00/00 00:00\0";

	while (1) {

		xQueueReceive(queueDSD, &data, portMAX_DELAY);

		memcpy(ty, cadenaSD, 125);
		memcpy(cadenaSD, ty + 24, 95);
		memcpy(cadenaSD + 96, data, 23);

//	  HAL_UART_Transmit(&huart1, cadenaSD , sizeof( cadenaSD  ),1000);

		xQueueSendToBack(queueSD, &cadenaSD, portMAX_DELAY);

	}

}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_TIM1_Init();
	MX_SPI1_Init();
	MX_FATFS_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	MX_GPIO_Init();
	//MX_USART2_UART_Init();
	MX_I2C1_Init();
	HD44780_Init(2);
	HD44780_Clear();
	HD44780_SetCursor(0, 0);
	HD44780_PrintStr("      TD_3      ");
	HD44780_SetCursor(0, 1);
	HD44780_PrintStr("    PARCIAL1    ");
	TIM1->ARR = 0x708;    //
	TIM1->CCR1 = 0xB4;    //

	// Creo el semaforo de la tarea handler por interrupcion
	vSemaphoreCreateBinary(sem_UART1);
	//vSemaphoreCreateBinary(sem_Dmfd);

	Mutex = xSemaphoreCreateMutex();

	xSemaphoreTake(sem_UART1, 0);

	// Creo la cola del adc, size 1 item
	//queueUART2 = xQueueCreate(1,sizeof(dataUART));
	queueUART1 = xQueueCreate(1, sizeof(dataUART));
	queuePWM = xQueueCreate(1, sizeof(dataPWM));
	queueMFD = xQueueCreate(1, sizeof(queueMFD_t));
	queueLCD = xQueueCreate(1, sizeof(queueLCD_t));
	queueDmfd = xQueueCreate(1, sizeof(queueS_1));
	queueSD = xQueueCreate(1, sizeof(cadenaSD_F));
	queueDSD = xQueueCreate(1, sizeof(dataUART));

	// Creo las tareas
	xTaskCreate(ReadUART1Task, "", configMINIMAL_STACK_SIZE + 300, NULL, 1,
			&h_UART1);
	xTaskCreate(PWMTask, "", configMINIMAL_STACK_SIZE, NULL, 1, &h_pwm);
	xTaskCreate(MFDTask, "", configMINIMAL_STACK_SIZE, NULL, 1, &h_mfd);
	xTaskCreate(LCDTask, "", configMINIMAL_STACK_SIZE, NULL, 1, &h_lcd);
	xTaskCreate(DMFDTask, "", configMINIMAL_STACK_SIZE, NULL, 1, &h_Dmfd);
	xTaskCreate(SDTask, "", configMINIMAL_STACK_SIZE + 1500, NULL, 2, &h_SD);
	xTaskCreate(DSDTask, "", configMINIMAL_STACK_SIZE, NULL + 1500, 1, &h_dsd);

	//xTaskCreate(presento1Task, "",configMINIMAL_STACK_SIZE,NULL, 1,(TaskHandle_t *) NULL);

	HAL_GPIO_WritePin(led_yy_GPIO_Port, led_yy_Pin, 1);
	HAL_Delay(2000);

	HAL_UART_Receive_IT(&huart1, (uint8_t*) &cadena, 1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	vTaskStartScheduler();
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
	sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, led_gg_Pin | led_yy_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : led_gg_Pin led_yy_Pin */
	GPIO_InitStruct.Pin = led_gg_Pin | led_yy_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM3 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM3) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
