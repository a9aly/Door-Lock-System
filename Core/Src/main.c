/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_1602.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define Buzzer_GPIO_Port GPIOA
#define Buzzr_Pin GPIO_PIN_4
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
	uint8_t pin [4] = {'1','1','1','1'};
	uint8_t user_entry [4];
	uint8_t inputORnew;
	uint8_t count_wrong_pins=0;
	uint8_t doorIsclosed = 1;		// 0 door is opened - 1 door is closed
	extern uint8_t distance;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint8_t read_mobile (void);
uint8_t compare_entry (uint8_t * pin,uint8_t * user_entry);
void start_buzzer(void);
void adjust_lock(uint8_t openORclose);
void delayMS (uint32_t value);
void distanceCalculation(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	lcd_init();
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	// Start the TIM Base generation
	HAL_TIM_Base_Start(&htim1);
	// Pull Trig output low for a little time
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,GPIO_PIN_RESET);
	HAL_Delay(10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* Ask User Whether He Wants To Enter Pin or Reset Pin */
		lcd_print_start();
		do{
			inputORnew = read_mobile();
		}while (inputORnew!='1' && inputORnew !='2');
		lcd_clr();
		
		/*Handling Pin input*/
		if(inputORnew=='1')
		{
			lcd_input_menu();
			for(int i=0; i<4;i=i+1)
			{
				user_entry [i] = read_mobile();
				lcd_gotoxy(i,1);
				lcd_puts("*");
			}
			
			if(compare_entry(pin,user_entry)=='t')
			{
				lcd_clr();
				lcd_gotoxy(0,0);
				lcd_puts("DOOR IS OPEN");
				adjust_lock(0);
				while(doorIsclosed ==0) distanceCalculation();
				count_wrong_pins=0;
			}
			else
			{
				lcd_clr();
				lcd_gotoxy(0,0);
				lcd_puts("Wrong Pin!");
				count_wrong_pins=count_wrong_pins+1;
				if(count_wrong_pins==3)(start_buzzer());
			}
		}
		
		/*Handling Pin Reset*/
		else if (inputORnew=='2')
		{
			lcd_gotoxy(0,0);
			lcd_puts("Enter PIN:");
			
			for(int i=0; i<4;i=i+1)
			{
				user_entry [i] = read_mobile();
				lcd_gotoxy(i,1);
				lcd_puts("*");
			}
			
			if(compare_entry(pin,user_entry)=='t')
			{
				lcd_clr();
				lcd_gotoxy(0,0);
				lcd_puts("PIN CAN BE");
				lcd_gotoxy(0,1);
				lcd_puts("4-NUMBERS ONLY");
				HAL_Delay(4000);
				lcd_clr();
				lcd_gotoxy(0,0);
				lcd_puts("Enter New Pin:");
				for(int i=0; i<4;i=i+1)
				{
					pin[i] = read_mobile();
					lcd_gotoxy(i,1);
					lcd_puts("*");
				}
			}
			else
			{
				lcd_clr();
				lcd_gotoxy(4,0);
				lcd_puts("Wrong Pin!");
				count_wrong_pins = count_wrong_pins+1;
				if(count_wrong_pins==3)(start_buzzer());
			}
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 800-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Trig_Pin|EN_Pin|BuzzerVDD_Pin|RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D7_Pin|D4_Pin|RW_Pin|D5_Pin
                          |D6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Trig_Pin EN_Pin BuzzerVDD_Pin RS_Pin */
  GPIO_InitStruct.Pin = Trig_Pin|EN_Pin|BuzzerVDD_Pin|RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D7_Pin D4_Pin RW_Pin D5_Pin
                           D6_Pin */
  GPIO_InitStruct.Pin = D7_Pin|D4_Pin|RW_Pin|D5_Pin
                          |D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* Function to take input from user through bluetooth connection*/
uint8_t read_mobile (void)
{
	uint8_t key = ' ';
	do{
		HAL_UART_Receive(&huart1,&key,sizeof(key),HAL_MAX_DELAY);
	}while(key<48 || key >57);
	return key;
}

/* Function to compare user input to the current pin */
uint8_t compare_entry (uint8_t * pin,uint8_t * user_entry)
{
	for (int i=0;i<4;i++)
	{
		if(pin[i] != user_entry[i])
			return 'f';
	}
	
	return 't';
}

/* Function to handle buzzer on/off */
void start_buzzer(void)
{
	 HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzr_Pin,GPIO_PIN_SET);
	 lcd_clr();
	 lcd_gotoxy(0,0);
	 lcd_puts("ENTER PIN");
	 lcd_gotoxy(0,1);
	 lcd_puts("TO DISABLE BUZZ");
	 HAL_Delay(4000);
	do{
		lcd_clr();
		lcd_gotoxy(0,0);
		lcd_puts("Enter Pin:");		
		for(int i=0; i<4;i=i+1)
		 {
				user_entry [i] = read_mobile();
				lcd_gotoxy(i,1);
				lcd_puts("*");
		 }
			if(compare_entry(pin,user_entry)=='t')
			{
				lcd_clr();
				lcd_gotoxy(0,0);
				lcd_puts("ALARM DISABLED");
				count_wrong_pins=0;
				HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzr_Pin,GPIO_PIN_RESET);
			}
		}while(compare_entry(pin,user_entry)=='f');
}

/*Function to adjust the lock *Servo Motor* */
/* Tim1 is set to fit 50HZ according to servo motor datasheet to rotate */
/* Adjust PWM of 0.11 (11%) open the lock */
/* Adjust PWM of 0.06 (6%) to exit the lock */
void adjust_lock(uint8_t openORclose)
{
	if(openORclose==0)
	{
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,11);
		doorIsclosed = 0;
	}
	else
	{
	 __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,6);
		doorIsclosed = 1;
	}
}

/* Timer 2 works on ms */
void delayMS (uint32_t value)
{
	// Set the TIM Counter Register value to 0
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	/*TIM Counter Register value is less than the intended value */
	while(__HAL_TIM_GET_COUNTER(&htim1)<value);
}

/* Calculate Distance From Object*/
void distanceCalculation(void)
{
		lcd_gotoxy(0,0);
		lcd_puts("DOOR CLOSE IN:");
		lcd_gotoxy(0,1);
		lcd_puts("Remaining is: 10");
		for(int i=1;i<=50;i++)
		{
			// pull TRIG high
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,GPIO_PIN_SET);
			//create 10µS delay 
			delayMS(10);
			// pull TRIG low
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,GPIO_PIN_RESET);
			// start TIM Input Capture measurement in interrupt mode
			HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
			// wait for a little time
			HAL_Delay(200);
			if(distance<100) break;	//someone passed the door conunt again
			lcd_gotoxy(0,1);
			// Print Remaining Time
			if(i%100==10)
				lcd_puts("Remaining is: 08");
			else if(i%100==20)
				lcd_puts("Remaining is: 06");
			else if(i%100==30)
				lcd_puts("Remaining is: 04");
			else if(i%100==40)
				lcd_puts("Remaining is: 02");
			else if(i%100==50)
				lcd_puts("Remaining is: 00");
			// Reached 10 seconds without interference - close the door -
			if(i==50)  adjust_lock(1);
		}
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
