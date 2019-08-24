/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VL53L1X_INT_Pin GPIO_PIN_4

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)  //prototype cho ham printf()

// People Counting defines
#define NOBODY 0
#define SOMEONE 1
#define LEFT 0
#define RIGHT 1

#define DIST_THRESHOLD_MAX  1780

#define isInterrupt 1 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t	dev=0x52;   //dia chi cua VL53L1X
int status = 0;       //kiem tra loi
volatile int IntCount;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Su dung ham printf() cho UART

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
	return ch;
}

//Callback() cho External Interru[t
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin==VL53L1X_INT_Pin)
	{
		IntCount++;
	}
}

//Ham tinh nguoi voi tham so la khoang cach va vung(Trai or Phai)
int ProcessPeopleCountingData(int16_t Distance, uint8_t zone) {
    static int PathTrack[] = {0,0,0,0};
    static int PathTrackFillingSize = 1; // init this to 1 as we start from state where nobody is any of the zones
    static int LeftPreviousStatus = NOBODY;  
    static int RightPreviousStatus = NOBODY;
    static int PeopleCount = 0;

    int CurrentZoneStatus = NOBODY;  //SOMEONE 
    int AllZonesCurrentStatus = 0;    // Kiem tra xem dang nam trong vung nao thong qua trong so tung vung
    int AnEventHasOccured = 0;  // = 1 thi co su kien tu vung nay sang vung khac, = 0 thi khong co su kien

	if (Distance < DIST_THRESHOLD_MAX) {
		// Someone is in !
		CurrentZoneStatus = SOMEONE;
	}

	// left zone
	if (zone == LEFT) {

		if (CurrentZoneStatus != LeftPreviousStatus)   // Neu (CurrentZoneStatus == LeftPreviousStatus == SOMEONE) thi bo qua
		{
			// event in left zone has occured
			AnEventHasOccured = 1;  

			if (CurrentZoneStatus == SOMEONE) {
				AllZonesCurrentStatus += 1;          //Vung LEFT co trong so la 1
			}
			// need to check right zone as well ...
			if (RightPreviousStatus == SOMEONE)    //Kiem tra no co nam trong vung giao khong
			{
				// event in left zone has occured
				AllZonesCurrentStatus += 2;
			}
			// remember for next time
			LeftPreviousStatus = CurrentZoneStatus;
		}
	}
	// right zone
	else {

		if (CurrentZoneStatus != RightPreviousStatus)  // Neu (CurrentZoneStatus == LeftPreviousStatus == SOMEONE) thi bo qua
		{

			// event in left zone has occured
			AnEventHasOccured = 1;
			
			if (CurrentZoneStatus == SOMEONE) {
				AllZonesCurrentStatus += 2;         //Vung RIGHT co trong so la 2
			}
			// need to left right zone as well ...
			if (LeftPreviousStatus == SOMEONE) 		//Kiem tra no co nam trong vung giao khong
			{
				// event in left zone has occured
				AllZonesCurrentStatus += 1;
			}
			// remember for next time
			RightPreviousStatus = CurrentZoneStatus;
		}
	}

	// if an event has occured
	if (AnEventHasOccured) {
		if (PathTrackFillingSize < 4) {
			PathTrackFillingSize ++;
		}

		// if nobody anywhere lets check if an exit or entry has happened
		if ((LeftPreviousStatus == NOBODY) && (RightPreviousStatus == NOBODY)) {

			// check exit or entry only if PathTrackFillingSize is 4 (for example 0 1 3 2) and last event is 0 (nobobdy anywhere)
			if (PathTrackFillingSize == 4) {
				// check exit or entry. no need to check PathTrack[0] == 0 , it is always the case

				if ((PathTrack[1] == 1)  && (PathTrack[2] == 3) && (PathTrack[3] == 2)) {
					// This an entry
					PeopleCount ++;

				} else if ((PathTrack[1] == 2)  && (PathTrack[2] == 3) && (PathTrack[3] == 1)) {
					// This an exit
					PeopleCount --;
				}
			}

			PathTrackFillingSize = 1;
		}
		else {
			// update PathTrack
			// example of PathTrack update
			// 0
			// 0 1
			// 0 1 3
			// 0 1 3 1
			// 0 1 3 3
			// 0 1 3 2 ==> if next is 0 : check if exit
			PathTrack[PathTrackFillingSize-1] = AllZonesCurrentStatus;
		}
	}

	// output debug data to main host machine
	return(PeopleCount);

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int io_2v8 = 1;  //2V8 mode
  uint8_t byteData, sensorState=0;
  uint16_t wordData;
  uint16_t Distance;
  uint8_t RangeStatus;
  uint8_t dataReady;
  int center[2] = {167,231}; /* these are the SPAD center of the 2 8*16 zones */
  int Zone = 0;
  int PplCounter = 0;
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
  /* USER CODE BEGIN 2 */
	
	// Those basic I2C read functions can be used to check your own I2C functions */
	
  status = VL53L1_RdByte(dev, 0x010F, &byteData);
  printf("VL53L1X Model_ID: %X\n", byteData);
  status = VL53L1_RdByte(dev, 0x0110, &byteData);
  printf("VL53L1X Module_Type: %X\n", byteData);
  status = VL53L1_RdWord(dev, 0x010F, &wordData);
  printf("VL53L1X: %X\n", wordData);
  while (sensorState == 0) 
  {
      status = VL53L1X_BootState(dev, &sensorState);
      HAL_Delay(2);
  }
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
  printf("Chip booted\n");
  /* Initialize and configure the device according to people counting need */
  status = VL53L1X_SensorInit(dev);
	if (io_2v8)  //2V8 mode
  {
		uint8_t i;
		status = VL53L1_RdByte(dev, 0x002E, &i);
		if (status == 0) {
			i = (i & 0xfe) | 0x01;
			status = VL53L1_WrByte(dev, 0x002E,i);
		}
  }
  status += VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
  status += VL53L1X_SetTimingBudgetInMs(dev, 50); /* in ms possible values [20, 50, 100, 200, 500] */
  status += VL53L1X_SetInterMeasurementInMs(dev, 54);
  status += VL53L1X_SetROI(dev, 8, 16); /* minimum ROI 4,4 */
  if (status != 0) 
	{
      printf("Error in Initialization or configuration of the device\n");
      return (-1);
  }
  printf("Start counting people  ...\n");
  status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* read and display data */
    while (dataReady == 0) 
		{
			status = VL53L1X_CheckForDataReady(dev, &dataReady);
			HAL_Delay(2);
    }

		dataReady = 0;
		status += VL53L1X_GetRangeStatus(dev, &RangeStatus);
		status += VL53L1X_GetDistance(dev, &Distance);
		status += VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
		if (status != 0) {
			printf("Error in operating the device\n");
			return (-1);
		}

		// wait a couple of milliseconds to ensure the setting of the new ROI center for the next ranging is effective
		// otherwise there is a risk that this setting is applied to current ranging (even if timing has expired, the intermeasurement
		// may expire immediately after.
		HAL_Delay(10);
		status = VL53L1X_SetROICenter(dev, center[Zone]);
		if (status != 0) {
			printf("Error in chaning the center of the ROI\n");
			return (-1);
		}

		// inject the new ranged distance in the people counting algorithm
		PplCounter = ProcessPeopleCountingData(Distance, Zone);

		Zone++;
		Zone = Zone%2;

		printf("%d, %d, %d, %d\n", Zone, RangeStatus, Distance, PplCounter);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
