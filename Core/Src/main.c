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
#include <stdio.h>
#include <string.h>
#include "lidar_model_config.h"
#include "ai_platform.h"
#include "lidar_model.h"
#include "lidar_model_data.h"
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
CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char RXBuf[50];
ai_handle lidar_model;
float aiInData[AI_LIDAR_MODEL_IN_1_SIZE];
float aiOutData[AI_LIDAR_MODEL_OUT_1_SIZE];
ai_u8 activations[AI_LIDAR_MODEL_DATA_ACTIVATIONS_SIZE];
ai_buffer *ai_input;
ai_buffer *ai_output;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
static void AI_Init(void);
static void AI_Run(float *pIn, float *pOut);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void AI_Init(void)
{
	ai_error err;

	/* Create a local array with the addresses of the activations buffers */
	const ai_handle act_addr[] = { activations };
	/* Create an instance of the model */
	err = ai_lidar_model_create_and_init(&lidar_model, act_addr, NULL);
	if (err.type != AI_ERROR_NONE) {
//		printf("ai_network_create error - type=%d code=%d\r\n", err.type, err.code);
		Error_Handler();
	}
	ai_input = ai_lidar_model_inputs_get(lidar_model, NULL);
	ai_output = ai_lidar_model_outputs_get(lidar_model, NULL);
}

static void AI_Run(float *pIn, float *pOut)
{
	ai_i32 batch;
	ai_error err;

	/* Update IO handlers with the data payload */
	ai_input[0].data = AI_HANDLE_PTR(pIn);
	ai_output[0].data = AI_HANDLE_PTR(pOut);
	/* Run the network */
	batch = ai_lidar_model_run(lidar_model, ai_input, ai_output);
	if (batch != 1) {
		err = ai_lidar_model_get_error(lidar_model);
//		printf("AI ai_network_run error - type=%d code=%d\r\n", err.type, err.code);
		Error_Handler();
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t timestamp;		// For the interference duration
	uint32_t duration;
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
  MX_USART1_UART_Init();
  MX_CRC_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  snprintf(RXBuf, 50, "%s", "Start Interfacing \n");
  HAL_UART_Transmit(&huart1, (uint8_t*)RXBuf, sizeof(RXBuf), 100);
  // Start timer/counter
  HAL_TIM_Base_Start(&htim11);

  //Initialize neural network
  AI_Init();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Fill Data Input
	  	  float dataDump[] = {5.888865347658078,0,0,0,111.10900872656761,109.05510104045345,108.02071849821927,107.00280918718755,106.00144873875115,105.01670255576785,104.04862516164646,103.09725953891206,103.16130796257711,103.24238864154488,103.34101254014037,103.4576980512243,104.59329824732144,104.74854770575715,104.92364686056774,106.12264764797895,107.34552275569293,108.59345859467574,107.85061884703461,105.10584142016687,104.39666151711577,106.75214792642562,108.12456635380869,109.5280679314614,0,5.01854064183178,0,0,0,0,0,0,0,4.860095636738915,0,4.817967539367538,4.796499408727395,4.774754955371732,0,0,4.707809259236121,4.68490439654601,4.6616949164806085,0,4.6143374695147,0,129.19204174763107,129.21340485906933,128.05942709870376,125.69477828955597,123.30020945397283,122.10358869104591,122.14029821718283,122.19110105540491,122.2553139297535,123.61739961617022,125.0208152927155,126.46622127017896,127.9542909051835,129.48571166589255,131.06118624099184,4.15358660422704,4.12274195624377,0,4.059611147340039,0,3.9944822826865973,0,0,3.89283008838383,3.857838560808834,3.822270492843074,0,3.7493467844483783,0,3.673938123739504,3.637341404983321,21.4837263160774,34.376005432398074,37.16812284932135,35.079749417036716,3.824182244219703,0,36.8057486436688,128.59082187412537,126.86613748235222,125.18569436267624,123.54872550549798,123.38797144406767,121.81716442966959,4.124398943250895,0,0,4.215471727213927,4.2449260761503815,0,0,4.330717398480374,4.3584882256786885,0,4.412843412128965,123.26676221852127,120.79803093340597,119.59906582572016,118.43890499762655,117.31693400309831,116.23254460413912,115.1851340913222,114.17410465370159,114.34121350648587,114.52506882598327,114.72619819194193,114.94510655772227,116.28997197202574,119.83795189018991,122.26842388684635,122.52052564479493,120.63712652154473,120.94630731266078,122.34361644561679,4.881860846001078,0,0,117.60262961675522,115.98994800158142,114.41648837558262,113.91250819576472,111.38136006465773,107.86810994917397,5.056795246213487,5.075091976714749,0,5.111048800188025,5.128716234552549,0,5.163446874326768,5.180516928936709,0,5.21408305483923,0,5.246905076711544,5.263044738296187,0,5.294796155206137,0,5.325862675838405,0,5.356266224145301,5.371225980405345,106.00290845880805,106.0196889675595,106.05098002917522,105.09323674430871,105.15100800802512,105.22179763173466,105.30517102764806};
	  	  for(uint32_t i = 0; i < AI_LIDAR_MODEL_IN_1_SIZE; i++)
	  	  	  {
	  	  		  aiInData[i] = (float)dataDump[i];
	  	  	  }

	  	  snprintf(RXBuf, 50, "%s", "Run AI -----------------------------------\n");
	  	  HAL_UART_Transmit(&huart1, (uint8_t*)RXBuf, sizeof(RXBuf), 100);

	  	  timestamp = htim11.Instance->CNT;

	  	  // Run AI
	  	  AI_Run(aiInData, aiOutData);

	  	  duration = htim11.Instance->CNT - timestamp;

	  	  snprintf(RXBuf, 50, "%s", "Output -----------------------------------\n");
	      HAL_UART_Transmit(&huart1, (uint8_t*)RXBuf, sizeof(RXBuf), 100);
	  	  for(uint32_t i = 0; i < AI_LIDAR_MODEL_OUT_1_SIZE; i++)
	  		  {
	  			  snprintf(RXBuf, 50, "%f", aiOutData[i]);
	  			  HAL_UART_Transmit(&huart1, (uint8_t*)RXBuf, sizeof(RXBuf), 100);
	  		  }

	  	  snprintf(RXBuf, 50, "Duration : %d", (uint8_t)duration);
	  	  HAL_UART_Transmit(&huart1, (uint8_t*)RXBuf, sizeof(RXBuf), 100);
	  	  HAL_Delay(1000);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 100-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
