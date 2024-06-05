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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "usbd_customhid.h"
#define VREF_CAL *(_IO uint16_t*) ()
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern USBD_HandleTypeDef hUsbDeviceFS;
uint8_t USBdataFlag;
uint8_t USBRxBuff[64];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef union
{
	struct
	{
		unsigned char low_byte;
		unsigned char mlow_byte;
		unsigned char mhigh_byte;
		unsigned char high_byte;
	}float_byte;
 
	float  value;
}floatUnionType;


int numbers[10] = {0x0e62,0X0202,0x0e28,0x062a,0x024a,0x046a,0x0c6a,0x0222,0x0e6a,0x066a};
int chars[] = {0x0c68,0x0a6a};
//int numbers[10] = {0x0ff2,0x0392,0x0fb8,0x07ba,0x03da,0x05fa,0x0dfa,0x03b2,0x0ffa,0x07fa};
int count = 0;
void func1(int c){
	switch(c){
		case 0:
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
		break;
		case 1:
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
		break;
		case 2:
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
		break;
		default:
			count = 0;
		break;
	}
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIOB,0x0190,GPIO_PIN_SET);
}

void func2(int c){
	switch(c){
		case 0:
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
		break;
		case 1:
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
		break;
		case 2:
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
		break;
		default:
			count = 0;
		break;
	}
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIOB,0x0190,GPIO_PIN_SET);
}

void show3(double num){
	HAL_GPIO_WritePin(GPIOB,0x0e6a,GPIO_PIN_RESET);
	if(num>0&&num<100){
		if(num<10&&num>1){
			HAL_GPIO_WritePin(GPIOB,numbers[(int)num],GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
			func1(2);
			HAL_GPIO_WritePin(GPIOB,0x0e6a,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,numbers[((int)(num*10))%10],GPIO_PIN_SET);
			
			func1(1);
			HAL_GPIO_WritePin(GPIOB,0x0e6a,GPIO_PIN_RESET);
			
			HAL_GPIO_WritePin(GPIOB,numbers[((int)(num*100))%10],GPIO_PIN_SET);
			func1(0);
		}else if(num>10){
			HAL_GPIO_WritePin(GPIOB,numbers[(int)(num/10)],GPIO_PIN_SET);
			func1(2);
			HAL_GPIO_WritePin(GPIOB,0x0e6a,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,numbers[((int)num)%10],GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
			func1(1);
			HAL_GPIO_WritePin(GPIOB,0x0e6a,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,numbers[((int)(num*10))%10],GPIO_PIN_SET);
			func1(0);
		}else if(num<1){
			HAL_GPIO_WritePin(GPIOB,numbers[0],GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
			func1(2);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,0x0e6a,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,numbers[((int)(num*10))%10],GPIO_PIN_SET);
			func1(1);
			HAL_GPIO_WritePin(GPIOB,0x0e6a,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,numbers[((int)(num*100))%10],GPIO_PIN_SET);
			func1(0);
		}
	}else if(num<0.0000001){
			HAL_GPIO_WritePin(GPIOB,numbers[0],GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
			func1(2);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,0x0e6a,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,numbers[0],GPIO_PIN_SET);
			func1(1);
			HAL_GPIO_WritePin(GPIOB,0x0e6a,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,numbers[0],GPIO_PIN_SET);
			func1(0);
	}else{
			HAL_GPIO_WritePin(GPIOB,chars[0],GPIO_PIN_SET);
			func1(2);
			HAL_GPIO_WritePin(GPIOB,0x0e6a,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,chars[1],GPIO_PIN_SET);
			func1(1);
			HAL_GPIO_WritePin(GPIOB,0x0e6a,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,chars[1],GPIO_PIN_SET);
			func1(0);
	}
}

void show4(double num){
	HAL_GPIO_WritePin(GPIOC,0x0e6a,GPIO_PIN_RESET);
	if(num>0&&num<100){
		if(num<10&&num>1){
			HAL_GPIO_WritePin(GPIOC,numbers[(int)num],GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
			func2(2);
			HAL_GPIO_WritePin(GPIOC,0x0e6a,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,numbers[((int)(num*10))%10],GPIO_PIN_SET);
			
			func2(1);
			HAL_GPIO_WritePin(GPIOC,0x0e6a,GPIO_PIN_RESET);
			
			HAL_GPIO_WritePin(GPIOC,numbers[((int)(num*100))%10],GPIO_PIN_SET);
			func2(0);
		}else if(num>10){
			HAL_GPIO_WritePin(GPIOC,numbers[(int)(num/10)],GPIO_PIN_SET);
			func2(2);
			HAL_GPIO_WritePin(GPIOC,0x0e6a,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,numbers[((int)num)%10],GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
			func2(1);
			HAL_GPIO_WritePin(GPIOC,0x0e6a,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,numbers[((int)(num*10))%10],GPIO_PIN_SET);
			func2(0);
		}else if(num<1){
			HAL_GPIO_WritePin(GPIOC,numbers[0],GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
			func2(2);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,0x0e6a,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,numbers[((int)(num*10))%10],GPIO_PIN_SET);
			func2(1);
			HAL_GPIO_WritePin(GPIOC,0x0e6a,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,numbers[((int)(num*100))%10],GPIO_PIN_SET);
			func2(0);
		}
	}else if(num<0.0000001){
			HAL_GPIO_WritePin(GPIOC,numbers[0],GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
			func2(2);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,0x0e6a,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,numbers[0],GPIO_PIN_SET);
			func2(1);
			HAL_GPIO_WritePin(GPIOC,0x0e6a,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,numbers[0],GPIO_PIN_SET);
			func2(0);
	}else{
			HAL_GPIO_WritePin(GPIOC,chars[0],GPIO_PIN_SET);
			func2(2);
			HAL_GPIO_WritePin(GPIOC,0x0e6a,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,chars[1],GPIO_PIN_SET);
			func2(1);
			HAL_GPIO_WritePin(GPIOC,0x0e6a,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,chars[1],GPIO_PIN_SET);
			func2(0);
	}
}


HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length);
unsigned short val[9];//??????DMA????????
float vii[8];//???????????
	void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	for(int8_t i = 0; i < 2; i++) {
	vii[i] = (val[i] / 4095.0) * 3.274; //?????:3.274V	
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	uint8_t vol[64]={0};
	floatUnionType fu[16];
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_ADCEx_Calibration_Start(&hadc1);
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)val,7);
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		for(int i =0;i<7;i++){
			fu[i].value=(float)val[i];
		}
		
		for(int i=0;i<7;i++){
			vol[i*4] = fu[i].float_byte.low_byte;
			vol[i*4+1] = fu[i].float_byte.mlow_byte;
			vol[i*4+2] = fu[i].float_byte.mhigh_byte;
			vol[i*4+3] = fu[i].float_byte.high_byte;
		}
		
		for(int p =0;p<100;p++){
			
				show4(fu[USBRxBuff[0]].value/4096*3.3*10);

			
		}
			
		if(USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, vol, 64) == USBD_OK)
	{
		
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC3 PC4
                           PC5 PC6 PC7 PC8
                           PC9 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11
                           PB3 PB4 PB5 PB6
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
