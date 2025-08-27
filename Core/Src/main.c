/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include"math.h"
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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int voltage2 = 0;
int voltage1 = 0;
int voltage3 = 0;

int Ic = 0;
int Ib = 0;

double Iac = 0.0;
double Iab = 0.0;

int cnt = 0;

int B = 0;
int B3 = 0;

int volt2 = 0;
int volt1 = 0;
int volt3 = 0;
int flag = 0;

int difference = 0;
int difference1 = 0;
int difference2 = 0;
int threshold = 868;

int row = 0;
int col = 0;

char npn0[] = "BJT Is NPN EBC    ";
char npn1[] = "BJT Is NPN CBE    ";
char pnp0[] = "BJT IS PNP  EBC   ";
char pnp1[] = "BJT IS PNP  CBE   ";
char error[] = "There is not BJT ";
char er[] = "ERROR";

char B1[20] = "";
char B2[20] = "";
char volta[20] = "";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t Read_ADC(uint32_t channel) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = channel;
	sConfig.Rank = 1;

	// ADC kanalını seç ve başlat
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return adc_value;
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
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	lcd_init();

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		flag = 0;
		// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);

		HAL_Delay(100);

		voltage2 = Read_ADC(ADC_CHANNEL_0);
		volt2 = (voltage2 / 1023 * 5.0);

		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);

		HAL_Delay(100);

		voltage3 = Read_ADC(ADC_CHANNEL_8);

		difference = abs(voltage3 - voltage2);

		if (difference < threshold) {
			HAL_UART_Transmit(&huart2, error, strlen(error), 100);
			lcd_put_cur(0, 0);
			lcd_send_string(error);
			HAL_Delay(300);
		}

		//BJT İS NPN
		else if (voltage3 > voltage2) {

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);

			HAL_Delay(100);

			voltage2 = Read_ADC(ADC_CHANNEL_0);
			volt2 = (voltage2 / 1023 * 5.0);

			voltage3 = Read_ADC(ADC_CHANNEL_8);
			volt3 = (voltage3 / 1023 * 5.0);

			difference1 = abs(voltage3 - voltage2); //voltage difference between pin2 and pin3

			HAL_Delay(100);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);

			HAL_Delay(100);

			voltage1 = Read_ADC(ADC_CHANNEL_1);
			volt1 = (voltage1 / 1023 * 5.0);

			voltage2 = Read_ADC(ADC_CHANNEL_0);
			volt2 = (voltage2 / 1023 * 5.0);

			difference2 = abs(voltage2 - voltage1); //voltage difference between pin1 and pin2

			if (difference1 > difference2) {
				HAL_UART_Transmit(&huart2, npn1, strlen(npn1), 100);  //npn cbe

				lcd_put_cur(0, 0);
				lcd_send_string(npn1);
				HAL_Delay(300);
//**********************************************************
				//b HESABI için confg
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
				voltage1 = Read_ADC(ADC_CHANNEL_1);

				volt1 = (voltage1 / 4095 * 3.3);			 //vc

				voltage2 = Read_ADC(ADC_CHANNEL_0);
				volt2 = (voltage2 / 4095 * 3.3);			 //vb

				Ic = abs(voltage1 - 4095);
				Ib = abs(voltage2 - 4095);
				B = Ic / Ib;
				sprintf(B1, "Bs= %d", B);
				lcd_put_cur(1, 0);
				lcd_send_string(B1);

				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 31);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
				Ic = 0;
				Ib = 0;
				B3 = 0;
				while (1) {

					if (flag == 10) {
						break;
					}

					voltage2 = Read_ADC(ADC_CHANNEL_0);
					voltage1 = Read_ADC(ADC_CHANNEL_1);
					Ic = abs(voltage1 - 4095);
					Ib = abs(voltage2 - 1241);
					B3 = Ic / Ib;
					sprintf(B2,
							"Ba= %d voltage2=%d    voltateg1=%d   Ib=%d  ıc=%d\n\r",
							B3, voltage2, voltage1, Ib, Ic);
					sprintf(volta, "Ba= %d   ", B3);
					lcd_put_cur(1, 6);
					lcd_send_string(volta);
					HAL_UART_Transmit(&huart2, B2, strlen(B2), 1000);
					flag++;
					HAL_Delay(1000);

				}
				//***********************************************************
			}

			else if (difference1 <= difference2) {
				HAL_UART_Transmit(&huart2, npn0, strlen(npn0), 100);//npn ebc
				// lcd_init ();
				//lcd_clear();
				lcd_put_cur(0, 0);
				lcd_send_string(npn0);
				HAL_Delay(300);
				//**********************************************************
				//b HESABI için confg
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
				// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
				voltage1 = Read_ADC(ADC_CHANNEL_8);

				volt1 = (voltage1 / 4095 * 3.3);			 				//vc

				voltage2 = Read_ADC(ADC_CHANNEL_0);
				volt2 = (voltage2 / 4095 * 3.3);			 				//vb

				Ic = abs(voltage1 - 4095);
				Ib = abs(voltage2 - 4095);
				B = Ic / Ib;
				sprintf(B1, "Bs= %d", B);
				lcd_put_cur(1, 0);
				lcd_send_string(B1);
				//lcd_send_data(B);

				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 31);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
				Ic = 0;
				Ib = 0;
				B3 = 0;

				while (1) {

					if (flag == 10) {
						break;
					}

					voltage2 = Read_ADC(ADC_CHANNEL_0);

					//	volt2 = (voltage2/4095*3.3);//vb

					voltage1 = Read_ADC(ADC_CHANNEL_8);

					//volt1 = (voltage1/4095*3.3);//vc

					Ic = abs(voltage1 - 4095);
					Ib = abs(voltage2 - 1241);
					//									 Iac=Ic/2000;

					B3 = Ic / Ib;

					// B3=Ic/Ib;
					sprintf(B2,
							"Ba= %d voltage2=%d    voltateg1=%d   Ib=%d  ıc=%d\n\r",
							B3, voltage2, voltage1, Ib, Ic);
					sprintf(volta, "Ba= %d    ", B3);
					lcd_put_cur(1, 8);
					lcd_send_string(volta);
					HAL_UART_Transmit(&huart2, B2, strlen(B2), 1000);

					HAL_Delay(1000);

					flag++;

				}

			}

		}
		//bjt pnp
		else if (voltage3 < voltage2) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);

			HAL_Delay(100);

			voltage2 = Read_ADC(ADC_CHANNEL_0);
			// volt2 = (voltage2/1023*5.0);
			voltage1 = Read_ADC(ADC_CHANNEL_1);

			// volt1 = (voltage1/1023*5.0);
			difference1 = abs(voltage1 - voltage2); //voltage difference between pin2 and pin1

			HAL_Delay(100);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);

			HAL_Delay(100);

			voltage3 = Read_ADC(ADC_CHANNEL_8);
			// volt3 = (voltage3/1023*5.0);

			voltage2 = Read_ADC(ADC_CHANNEL_0);
			//volt2 = (voltage2/1023*5.0);

			difference2 = abs(voltage2 - voltage3); //voltage difference between pin3 and pin2

			if (difference1 > difference2) {
				HAL_UART_Transmit(&huart2, pnp0, strlen(pnp0), 100);
				// lcd_init ();
				//lcd_clear();
				lcd_put_cur(0, 0);
				lcd_send_string(pnp0);
				HAL_Delay(300);
				//****************************
				//b HESABI için confg
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
				// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
				voltage1 = Read_ADC(ADC_CHANNEL_8);

				volt1 = (voltage1 / 4095 * 3.3);							//vc

				voltage2 = Read_ADC(ADC_CHANNEL_0);
				volt2 = (voltage2 / 4095 * 3.3);							//vb

				Ic = abs(voltage1 - 0);
				Ib = abs(voltage2 - 0);
				B = Ic / Ib;
				sprintf(B1, "Bs= %d", B);
				lcd_put_cur(1, 0);
				lcd_send_string(B1);
				//lcd_send_data(B);

				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 62);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
				Ic = 0;
				Ib = 0;
				B3 = 0;
				while (1) {

					if (flag == 10) {
						break;
					}

					voltage2 = Read_ADC(ADC_CHANNEL_0);

					//	volt2 = (voltage2/4095*3.3);//vb

					voltage1 = Read_ADC(ADC_CHANNEL_8);

					//volt1 = (voltage1/4095*3.3);//vc

					Ic = abs(voltage1 - 0);
					Ib = abs(voltage2 - 2482);
					//									 Iac=Ic/2000;

					B3 = Ic / Ib;

					// B3=Ic/Ib;
					sprintf(B2,
							"Ba= %d voltage2=%d    voltateg1=%d   Ib=%d  ıc=%d\n\r",
							B3, voltage2, voltage1, Ib, Ic);
					sprintf(volta, "Ba= %d    ", B3);
					lcd_put_cur(1, 6);
					lcd_send_string(volta);
					HAL_UART_Transmit(&huart2, B2, strlen(B2), 1000);
					flag++;
					HAL_Delay(1000);

				}
				//********************
			} else if (difference1 <= difference2) {
				HAL_UART_Transmit(&huart2, pnp1, strlen(pnp1), 100);
				//lcd_init ();
				// lcd_clear();
				lcd_put_cur(0, 0);
				lcd_send_string(pnp1);
				HAL_Delay(300);
				//****************************
				//b HESABI için confg
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
				// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
				voltage1 = Read_ADC(ADC_CHANNEL_1);

				volt1 = (voltage1 / 4095 * 3.3);							//vc

				voltage2 = Read_ADC(ADC_CHANNEL_0);
				volt2 = (voltage2 / 4095 * 3.3);							//vb

				Ic = abs(voltage1 - 0);
				Ib = abs(voltage2 - 0);
				B = Ic / Ib;
				sprintf(B1, "Bs= %d", B);
				lcd_put_cur(1, 0);
				lcd_send_string(B1);
				//lcd_send_data(B);

				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 62);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
				Ic = 0;
				Ib = 0;
				B3 = 0;
				while (1) {

					if (flag == 10) {
						break;
					}

					voltage2 = Read_ADC(ADC_CHANNEL_0);

					//	volt2 = (voltage2/4095*3.3);//vb

					voltage1 = Read_ADC(ADC_CHANNEL_1);

					//volt1 = (voltage1/4095*3.3);//vc

					Ic = abs(voltage1 - 0);
					Ib = abs(voltage2 - 2482);
					//									 Iac=Ic/2000;

					B3 = Ic / Ib;

					// B3=Ic/Ib;
					sprintf(B2,
							"Ba= %d voltage2=%d    voltateg1=%d   Ib=%d  ıc=%d\n\r",
							B3, voltage2, voltage1, Ib, Ic);
					sprintf(volta, "Ba= %d    ", B3);
					lcd_put_cur(1, 6);
					lcd_send_string(volta);
					HAL_UART_Transmit(&huart2, B2, strlen(B2), 1000);
					flag++;
					HAL_Delay(1000);

				}

			}

		}

		else {
			HAL_UART_Transmit(&huart2, er, strlen(er), 100);
			//lcd_init ();
			// lcd_clear();
			lcd_put_cur(0, 0);
			lcd_send_string(er);
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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
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
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
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

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 83;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 100;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

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
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin PA6 PA7 */
	GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
