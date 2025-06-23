/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
float adc1_value;
float ppm;
int check = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay(uint16_t t) {
	volatile unsigned long l = 0;
	for (uint16_t i = 0; i < t; i++) {
		for (l = 0; l < 6000; l++)
			;
	}
}

uint8_t usart1_recByte() {
	while ((USART1->SR & (1 << 5)) == 0)
		;
	return USART1->DR;
}

void USART1_Init(void) {
	RCC->APB2ENR |= (1 << 2) | (1 << 14); // Bật clock cho GPIOA và USART1

	/* Cấu hình chân TX (PA9) và RX (PA10) */
	GPIOA->CRH &= ~(0xFF << 4);  // Xóa cấu hình cũ
	GPIOA->CRH |= (0x0B << 4);    // PA9: AF Push-Pull, tốc độ cao
	GPIOA->CRH |= (0x04 << 8);    // PA10: Input pull-up
	GPIOA->ODR |= (1 << 10);      // Bật pull-up cho PA10

	/* Cấu hình USART1 */
	USART1->CR1 &= ~(1 << 13);   // Tắt USART1 trước khi cấu hình
	USART1->BRR = 7500;          // Baudrate = 72MHz / 9600
	USART1->CR1 = (1 << 2) | (1 << 3) | (1 << 13); // Bật RX, TX và USART1
}

void time_init_kitchen_door() {
	// Cấu hình clock cho GPIOA và TIM2
	RCC->APB1ENR |= (1 << 0);
	RCC->APB2ENR |= (1 << 2);

	TIM2->PSC = 1440 - 1;
	TIM2->ARR = 1000 - 1;
	// Cấu hình PA0 là alter function
	GPIOA->CRL &= ~(0xF << (4 * 0));
	GPIOA->CRL |= (0xB << (4 * 0));

	TIM2->CCMR1 &= ~(7 << 4);
	TIM2->CCMR1 |= (6 << 4);
	TIM2->CCMR1 |= (1 << 3);

	TIM2->CCER |= (1 << 0);
	TIM2->CR1 |= (1 << 0); // bật timer
}

void time_init_main_door() {
	RCC->APB1ENR |= (1 << 0);
	RCC->APB2ENR |= (1 << 2);

	TIM2->PSC = 1440 - 1;
	TIM2->ARR = 1000 - 1;

	GPIOA->CRL &= ~(0xF << (4 * 1));
	GPIOA->CRL |= (0xB << (4 * 1));

	TIM2->CCMR1 &= ~(7 << 12);
	TIM2->CCMR1 |= (6 << 12);
	TIM2->CCMR1 |= (1 << 11);
	TIM2->CCMR1 &= ~(3 << 8);

	TIM2->CCER |= (1 << 4);
	TIM2->CR1 |= (1 << 0); // bật timer
}

void time_init_bedroom_door() {
	RCC->APB1ENR |= (1 << 0);
	RCC->APB2ENR |= (1 << 2);

	TIM2->PSC = 1440 - 1;
	TIM2->ARR = 1000 - 1;

	GPIOA->CRL &= ~(0xF << (4 * 2));
	GPIOA->CRL |= (0xB << (4 * 2));

	TIM2->CCMR2 &= ~(7 << 4);
	TIM2->CCMR2 |= (6 << 4);
	TIM2->CCMR2 |= (1 << 3);
	TIM2->CCMR2 &= ~(3 << 0);

	TIM2->CCER |= (1 << 8);
	TIM2->CR1 |= (1 << 0);
}

void kitchen_door_on() {
	time_init_kitchen_door();
	TIM2->CCR1 = 80;
}

void kitchen_door_of() {
	time_init_kitchen_door();
	TIM2->CCR1 = 30;
}

void main_door_on() {
	time_init_main_door();
	TIM2->CCR2 = 30;
}

void main_door_of() {
	time_init_main_door();
	TIM2->CCR2 = 80;
}

void bedroom_door_on() {
	time_init_bedroom_door();
	TIM2->CCR3 = 30;
}

void bedroom_door_of() {
	time_init_bedroom_door();
	TIM2->CCR3 = 80;
}

void light_living_on() {
	GPIOB->ODR |= (1 << 0);
}

void light_living_of() {
	GPIOB->ODR &= ~(1 << 0);
}

void light_bedroom_on() {
	GPIOB->ODR |= (1 << 9);
}

void light_bedroom_of() {
	GPIOB->ODR &= ~(1 << 9);
}

void light_kitchen_on() {
	GPIOB->ODR |= (1 << 10);
}

void light_kitchen_of() {
	GPIOB->ODR &= ~(1 << 10);
}

void light_bathroom_on() {
	GPIOB->ODR |= (1 << 11);
}

void light_bathroom_of() {
	GPIOB->ODR &= ~(1 << 11);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	adc1_value = HAL_ADC_GetValue(&hadc1);
	float voltage = (adc1_value / 4096.0) * 5.0;
	float RS = ((5.0 - voltage) * 10.0) / voltage;
	float ratio = RS / 76.63;
	ppm = 105.19 * pow(ratio, -2.773);
	if (ppm > 1200) {
		GPIOB->BSRR |= (1 << 12);
		delay(100);
		GPIOB->BSRR |= (1 << 28);
		delay(100);
		time_init_kitchen_door();
		TIM2->CCR1 = 80;
		time_init_main_door();
		TIM2->CCR2 = 30;
		time_init_bedroom_door();
		TIM2->CCR3 = 30;
	} else {
		GPIOB->BSRR |= (1 << 28);
	}
	HAL_ADC_Start_IT(&hadc1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_7) {
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET) {
			GPIOB->ODR |= (1 << 1);
		} else {
			GPIOB->ODR &= ~(1 << 1);
		}

	}
}

void tivi_bat() {
	LCD_setCursor(0, 0);
	LCD_printf("Fetel HCMUS");
	LCD_setCursor(1, 0);
	LCD_printf("Welcome home");
}

void tivi_tat() {
	LCD_clear();
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
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	RCC->APB2ENR = 0xFC | (1 << 9);
	GPIOA->CRL = 0x44404444;
	GPIOB->CRH = 0x44433333;
	GPIOB->CRL = 0x33333433;
	LCD_init();
	LCD_clear();
	USART1_Init();
	time_init_kitchen_door();
	TIM2->CCR1 = 30;
	time_init_main_door();
	TIM2->CCR2 = 80;
	time_init_bedroom_door();
	TIM2->CCR3 = 80;
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_IT(&hadc1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		char signals = usart1_recByte();
		switch (signals) {
		case 'A':
			light_living_on();
			break;
		case 'B':
			light_living_of();
			break;
		case 'E':
			tivi_bat();
			break;
		case 'F':
			tivi_tat();
			break;
		case 'G':
			main_door_on();
			break;
		case 'H':
			main_door_of();
			break;
		case 'Y':
			kitchen_door_on();
			break;
		case 'Z':
			kitchen_door_of();
			break;
		case 'I':
			light_kitchen_on();
			break;
		case 'J':
			light_kitchen_of();
			break;
		case 'K':
			light_bedroom_on();
			break;
		case 'L':
			light_bedroom_of();
			break;
		case 'M':
			bedroom_door_on();
			break;
		case 'N':
			bedroom_door_of();
			break;
		case 'O':
			light_bathroom_on();
			break;
		case 'P':
			light_bathroom_of();
			break;
		case 'S':
			bedroom_door_on();
			main_door_on();
			kitchen_door_on();
			LCD_clear();
			LCD_setCursor(0,0);
			LCD_printf("SOS");
			delay(100);
			break;
		case 'U':
			bedroom_door_of();
			kitchen_door_of();
			main_door_of();
			LCD_clear();
			LCD_setCursor(1,0);
			LCD_printf("Safety");
			delay(100);
			break;
		}
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	D4_Pin | D5_Pin | D6_Pin | D7_Pin | RS_Pin | EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA6 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin
	 RS_Pin EN_Pin */
	GPIO_InitStruct.Pin = D4_Pin | D5_Pin | D6_Pin | D7_Pin | RS_Pin | EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
