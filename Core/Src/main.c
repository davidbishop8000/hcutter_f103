/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "ssd1306/ssd1306.h"
#include "ssd1306/ssd1306_tests.h"
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

typedef struct {
	GPIO_TypeDef *button_port;
	uint16_t button_pin;
	uint8_t short_state;
	uint8_t long_state;
	uint32_t time_key;
} StButtonsTypeDef;

StButtonsTypeDef stButtons[1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

enum MENU {
	MENU_START = 0,
	MENU_LENGTH,
	MENU_QUANT,
	MENU_MAX,
};

int curr_menu = 0;
int prog_start = 0;
int tube_length = 5;
int tube_quant = 5;
int tube_prev_quant = 0;

void servo_up()
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	TIM4->CCR1 = 20; //7 - 0, 12 - 45, 20 - 90, 32 - 180
	HAL_Delay(500);
	TIM4->CCR1 = 9;
	HAL_Delay(500);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
}

void servo_down()
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	TIM4->CCR1 = 20; //7 - 0, 12 - 45, 20 - 90, 32 - 180
	HAL_Delay(500);
	TIM4->CCR1 = 9;
	HAL_Delay(500);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
}

void motors_off()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	TIM4->CCR1 = 9;
	HAL_Delay(500);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
}

uint32_t move_timer = 0;
int state_prog = 0;
int off_state = 0;
int motors_stop = 0;

void move()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
}
void stop()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
}

void tube_cutting()
{
	off_state = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, !(GPIO_PinState)prog_start);
	if (prog_start)
	{
		if (state_prog == 0)
		{
			move_timer = HAL_GetTick();
			move();
			state_prog++;
		}
		else if (state_prog == 1)
		{
			if (HAL_GetTick() - move_timer > tube_length*30)
			{
				stop();
				move_timer = HAL_GetTick();
				HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
				TIM4->CCR1 = 20;
				state_prog++;
			}
		}
		else if (state_prog == 2)
		{
			if (HAL_GetTick() - move_timer > 500)
			{
				TIM4->CCR1 = 9;
				move_timer = HAL_GetTick();
				state_prog++;
			}
		}
		else if (state_prog == 3)
		{
			if (HAL_GetTick() - move_timer > 500)
			{
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
				state_prog = 0;
				tube_quant--;
				if (tube_quant<=0)
				{
					tube_quant = tube_prev_quant;
					prog_start = 0;
				}
			}
		}
	}
	else if (!motors_stop)
	{
		state_prog = 0;
		if (off_state == 0)
		{
			stop();
			HAL_Delay(10);
			move_timer = HAL_GetTick();
			TIM4->CCR1 = 9;
			off_state++;
		}
		else if (off_state == 1)
		{
			if (HAL_GetTick() - move_timer > 500)
			{
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
				off_state = 0;
				motors_stop = 1;
			}
		}
	}
}

void buttons_Init()
{
	stButtons[0].button_port = GPIOA;
	stButtons[0].button_pin = GPIO_PIN_6;
}

uint8_t short_state = 0;
uint8_t long_state = 0;
uint32_t time_key1 = 0;

void getButton()
{
	for (int i=0; i<1; i++)
	{
		uint32_t ms = HAL_GetTick();
		uint8_t key_state = HAL_GPIO_ReadPin(stButtons[i].button_port, stButtons[i].button_pin);
		if(key_state == 0 && !stButtons[i].short_state && (ms - stButtons[i].time_key) > 50)
		{
			stButtons[i].short_state = 1;
			stButtons[i].long_state = 0;
			stButtons[i].time_key = ms;
		}
		else if(key_state == 0 && !stButtons[i].long_state && (ms - stButtons[i].time_key) > 1000)
		{
			stButtons[i].long_state = 1;
			//long press
			if (i==0)
			{
				curr_menu++;
				if (curr_menu>=MENU_MAX)
				{
					curr_menu = 0;
				}
			}
		}
		else if(key_state == 1 && stButtons[i].short_state && (ms - stButtons[i].time_key) > 50)
		{
			stButtons[i].short_state = 0;
			stButtons[i].time_key = ms;

		  if(!stButtons[i].long_state)
		  {
			//short press
			  if (curr_menu == MENU_START)
			  {
				  prog_start = !prog_start;
			  }
			  else if (curr_menu == MENU_LENGTH)
			  {
				  tube_length++;
				  if (tube_length > 20) tube_length = 5;
			  }
			  else if (curr_menu == MENU_QUANT)
			  {
				  tube_quant+=5;
				  if (tube_quant > 50) tube_quant = 5;
				  tube_prev_quant = tube_quant;
			  }
		  }
		}
	}
}

void menu_update()
{
	SSD1306_COLOR color1 = White;
	SSD1306_COLOR color2 = White;
	SSD1306_COLOR color3 = White;
	if (curr_menu == MENU_START)
	{
		//char str [12];
		//if (!prog_start) snprintf(str, sizeof str, "%s", "pause");
		//else snprintf(str, sizeof str, "%s", "run");
		ssd1306_Fill(Black);
		color1 = Black;
		//ssd1306_SetCursor(3, 20);
		//ssd1306_WriteString(str, Font_7x10, White);
		for (int i=0; i<tube_quant; i++)
		{
			ssd1306_FillRectangle(4+i*3, 42, 5+i*3, 56, White);;
		}
	}
	else if (curr_menu == MENU_LENGTH) {
		ssd1306_Fill(Black);
		color2 = Black;
		//char str [12];
		//snprintf(str, sizeof str, "%d", tube_length);
		//ssd1306_SetCursor(57, 16);
		//ssd1306_WriteString(str, Font_11x18, White);
		ssd1306_FillRectangle(5, 42, tube_length*4, 56, White);
	}
	else if (curr_menu == MENU_QUANT)
	{
		ssd1306_Fill(Black);
		color3 = Black;
		//char str [12];
		//snprintf(str, sizeof str, "%d", tube_quant);
		//ssd1306_SetCursor(98, 16);
		//ssd1306_WriteString(str, Font_11x18, White);
		//ssd1306_DrawCircle(5+tube_quant*2, 49, 9, White);
		for (int i=0; i<tube_quant; i++)
		{
			ssd1306_DrawCircle(15+i*2, 49, 5, White);
		}
	}
	ssd1306_SetCursor(4, 2);
	ssd1306_WriteString("Status", Font_7x10, color1);
	ssd1306_SetCursor(47, 2);
	ssd1306_WriteString("Length", Font_7x10, color2);
	ssd1306_SetCursor(92, 2);
	ssd1306_WriteString("Quant", Font_7x10, color3);
	ssd1306_DrawRectangle(2, 0, 128, 63, White);
	ssd1306_Line(2, 13, 128, 13, White);
	ssd1306_Line(2, 35, 128, 35, White);
	ssd1306_Line(45, 1, 45, 35, White);
	ssd1306_Line(90, 1, 90, 35, White);
	static char strq [12];

	if (!prog_start) snprintf(strq, sizeof strq, "%s", "pause");
	else snprintf(strq, sizeof strq, "%s", "run");
	ssd1306_SetCursor(5, 20);
	ssd1306_WriteString(strq, Font_7x10, White);

	snprintf(strq, sizeof strq, "%d", tube_length);
	ssd1306_SetCursor(57, 16);
	ssd1306_WriteString(strq, Font_11x18, White);
	snprintf(strq, sizeof strq, "%d", tube_quant);
	ssd1306_SetCursor(98, 16);
	ssd1306_WriteString(strq, Font_11x18, White);

	ssd1306_UpdateScreen();
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_TIM4_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  buttons_Init();
  tube_prev_quant = tube_quant;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	  getButton();
	  if(curr_menu == MENU_START)
	  {
		  //getEncoder();
	  }
	  else if (curr_menu == MENU_LENGTH)
	  {
		  //read_bms_uart();
	  }
	  else if (curr_menu == MENU_QUANT)
	  {
	  	  //Moving();
	  }
	  menu_update();
	  tube_cutting();
	  HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 5624;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 255;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OLED_DC_Pin|OLED_Res_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|OLED_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_DC_Pin OLED_Res_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin|OLED_Res_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_CS_Pin */
  GPIO_InitStruct.Pin = OLED_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
