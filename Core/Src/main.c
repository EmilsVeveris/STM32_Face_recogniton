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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32_adafruit_lcd.h"
#include "lcd.h"
#include "stdio.h"
#include "math.h"
#include "ov9655.h"
#include "string.h"
#include "stdlib.h"
#include "ads7843.h"

#include "usbd_cdc_if.h"
#include "string.h"
#include "stream_buffer.h"
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
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

SRAM_HandleTypeDef hsram1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for getPicFromCam */
osThreadId_t getPicFromCamHandle;
const osThreadAttr_t getPicFromCam_attributes = {
  .name = "getPicFromCam",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sendPicToPy */
osThreadId_t sendPicToPyHandle;
const osThreadAttr_t sendPicToPy_attributes = {
  .name = "sendPicToPy",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for getCoordFromPy */
osThreadId_t getCoordFromPyHandle;
const osThreadAttr_t getCoordFromPy_attributes = {
  .name = "getCoordFromPy",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for showPicture */
osThreadId_t showPictureHandle;
const osThreadAttr_t showPicture_attributes = {
  .name = "showPicture",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Coordinates */
osMessageQueueId_t CoordinatesHandle;
const osMessageQueueAttr_t Coordinates_attributes = {
  .name = "Coordinates"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FSMC_Init(void);
static void MX_DCMI_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void *argument);
void StartGetPictureFromCamera(void *argument);
void StartSendPictureToPy(void *argument);
void StartGetCoordFromPy(void *argument);
void StartShowPicture(void *argument);

/* USER CODE BEGIN PFP */

int32_t cameraWriteReg(void *ctx, uint16_t reg, uint8_t *pdata, uint16_t length);
int32_t cameraReadReg(void *ctx, uint16_t reg, uint8_t *pdata, uint16_t length);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
	ITM_SendChar(ch);
	return(ch);
}

OV9655_Object_t cameraObject;
OV9655_IO_t	cameraIo;
ov9655_ctx_t cameraCtx;


uint32_t imgBuf[19200 + 1] = {0};
uint8_t syncBuf[] = {0, 255, 0, 255, 0, 255};

//uint8_t count = 0;

uint8_t frameDone = 0;
uint8_t recivedData = 0;
uint8_t ret = 0;
uint8_t *tempReciveBuf;

osThreadId_t tid;

StreamBufferHandle_t xStreamBuffer;
//uint8_t *data = "Hello world from a dumb person who can't follow a tutorial\n";

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
  MX_DMA_Init();
  MX_FSMC_Init();
  MX_DCMI_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_BCKL_GPIO_Port, LCD_BCKL_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CMR_RST_GPIO_Port, CMR_RST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CMR_SHDW_GPIO_Port, CMR_SHDW_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_BCKL_GPIO_Port, LCD_BCKL_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(CMR_RST_GPIO_Port, CMR_RST_Pin, GPIO_PIN_RESET);	// ACTIVE HIGH
  HAL_GPIO_WritePin(CMR_SHDW_GPIO_Port, CMR_SHDW_Pin, GPIO_PIN_RESET);	// ACTIVE HIGH    FUU

  BSP_LCD_Init();

  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_RED);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(10, 10, (uint8_t *)"testing", LEFT_MODE);
  BSP_LCD_FillRect(0, 0, 50, 50);


  cameraCtx.ReadReg = &cameraReadReg;
  cameraCtx.WriteReg = &cameraWriteReg;
  cameraObject.Ctx = cameraCtx;

  int32_t retInit = 0;
  retInit = OV9655_Init(&cameraObject, OV9655_R160x120, OV9655_RGB565);
  if(retInit != 0){
	//  BSP_LCD_DisplayStringAt(100, 100, (uint8_t *)"testing", LEFT_MODE);
	  while(1);
  }

  //HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)imgBuf, 19200/2);

  ADS7843_Init(&hspi2, 320, 240);

  tid = osThreadNew(StartShowPicture, NULL, NULL);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Coordinates */
  CoordinatesHandle = osMessageQueueNew (9, sizeof(uint8_t), &Coordinates_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  const size_t xStreamBufferSizeBytes = 100, xTriggerLevel = 10;
  	size_t xBytesSent;

  	const TickType_t x100ms = pdMS_TO_TICKS( 100 );
  	printf("Starting to create buffer \n");
  	    xStreamBuffer = xStreamBufferCreate( xStreamBufferSizeBytes, xTriggerLevel );

  	    if( xStreamBuffer == NULL )
  	    {
  	    	printf(" Error !!!  creating stream buffer \n");
  	    }
  	    else
  	    {

  			  printf("Succesfully created stream buffer \n");
  	    }
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of getPicFromCam */
  getPicFromCamHandle = osThreadNew(StartGetPictureFromCamera, NULL, &getPicFromCam_attributes);

  /* creation of sendPicToPy */
  sendPicToPyHandle = osThreadNew(StartSendPictureToPy, NULL, &sendPicToPy_attributes);

  /* creation of getCoordFromPy */
  getCoordFromPyHandle = osThreadNew(StartGetCoordFromPy, NULL, &getCoordFromPy_attributes);

  /* creation of showPicture */
  showPictureHandle = osThreadNew(StartShowPicture, NULL, &showPicture_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
/*
	  if(frameDone){
		  frameDone = 0;
		  //BSP_LCD_DrawRGB16Image(160, 120, 160, 120, (uint16_t *)imgBuf);
	  }

	  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)imgBuf, 19200);
	  HAL_Delay(100);

	  CDC_Transmit_FS(syncBuf, sizeof(syncBuf));
	  HAL_Delay(100);
	  for(uint16_t i = 0; i < 19200; i++){            // vcom shit
	  ret = 1;
	  while(ret){                                    // make sure it is transmitted
	  ret = CDC_Transmit_FS(((uint8_t *)imgBuf) + (i * 2), 2);    // send iamge over USB vcom
	  	  }
	  }

	  BSP_LCD_DrawRGB16Image(160, 120, 160, 120, (uint16_t *)imgBuf);
	  if(recivedData == 1)
	  {
		  BSP_LCD_DrawRect(160+tempReciveBuf[0], 120+tempReciveBuf[3], tempReciveBuf[1] - tempReciveBuf[3], tempReciveBuf[2] - tempReciveBuf[0]);
		  count = count +1;
	  	  if (count == 4)
	  	  {
	  		  recivedData = 0;
		  	  count = 0;
	  	  }
	  }

*/

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 1;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, MEMS_CS_Pin|CMR_RST_Pin|CMR_SHDW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_BCKL_Pin|LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MEMS_CS_Pin CMR_RST_Pin CMR_SHDW_Pin */
  GPIO_InitStruct.Pin = MEMS_CS_Pin|CMR_RST_Pin|CMR_SHDW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : TC_IRQ_Pin */
  GPIO_InitStruct.Pin = TC_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TC_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_BCKL_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_BCKL_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

int32_t cameraWriteReg(void *ctx, uint16_t reg, uint8_t *pdata, uint16_t length){
	int32_t ret = 1;

	uint8_t temp[32] = {0};
	memcpy(temp + 1, pdata, length);
	temp[0] = reg & 0xff;

	ret = HAL_I2C_Master_Transmit(&hi2c1, OV9655_ADD, temp, length + 1, 10000);


	return ret;
}

int32_t cameraReadReg(void *ctx, uint16_t reg, uint8_t *pdata, uint16_t length){
	int32_t ret = 0;

	uint8_t temp[32] = {0};
	temp[0] = reg & 0xff;

	ret = HAL_I2C_Master_Transmit(&hi2c1, OV9655_ADD, temp, 1, 100);
	ret = HAL_I2C_Master_Receive(&hi2c1, OV9655_ADD, pdata, length, 100);
	return ret;

}



//(ctx->handle, reg, data, length)
//HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, *imgBuf, 160*120);

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi){
	//HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	frameDone = 1;
}

void HAL_DCMI_HsyncCallback(DCMI_HandleTypeDef *hdcmi){
	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
}

void HAL_DCMI_VsyncCallback(DCMI_HandleTypeDef *hdcmi){
	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
}

void CDC_Recive_Callback(uint8_t *buf, uint32_t len)
{
	tempReciveBuf = buf;
	recivedData = 1;
	osThreadFlagsSet(tid, 0x0002U);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartGetPictureFromCamera */
/**
* @brief Function implementing the getPicFromCam thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetPictureFromCamera */
void StartGetPictureFromCamera(void *argument)
{
  /* USER CODE BEGIN StartGetPictureFromCamera */

  /* Infinite loop */
  for(;;)
  {
	  if(frameDone == 0)
	  {
		  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)imgBuf, 19200);
	  }
  }
  /* USER CODE END StartGetPictureFromCamera */
}

/* USER CODE BEGIN Header_StartSendPictureToPy */
/**
* @brief Function implementing the sendPicToPy thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendPictureToPy */
void StartSendPictureToPy(void *argument)
{
  /* USER CODE BEGIN StartSendPictureToPy */
  /* Infinite loop */
  for(;;)
  {
	  if(frameDone == 1){
		//  taskENTER_CRITICAL();
		  CDC_Transmit_FS(syncBuf, sizeof(syncBuf));
	 	//  HAL_Delay(100);
	 	  for(uint16_t i = 0; i < 19200; i++){            // vcom shit
	 	  ret = 1;
	 	  while(ret){                                    // make sure it is transmitted
	 	  ret = CDC_Transmit_FS(((uint8_t *)imgBuf) + (i * 2), 2);    // send iamge over USB vcom
	 	  }
	 	  }
	 	  frameDone = 0;
	 //	  taskEXIT_CRITICAL();
	  }
  }
  /* USER CODE END StartSendPictureToPy */
}

/* USER CODE BEGIN Header_StartGetCoordFromPy */
/**
* @brief Function implementing the getCoordFromPy thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetCoordFromPy */
void StartGetCoordFromPy(void *argument)
{
  /* USER CODE BEGIN StartGetCoordFromPy */
  /* Infinite loop */
  for(;;)
  {
	 /* void CDC_Recive_Callback(uint8_t *buf, uint32_t len)
	  {
	  	tempReciveBuf = buf;
	  	recivedData = 1;
	  	osThreadFlagsSet(tid, 0x0002U);
	  }*/
  }
  /* USER CODE END StartGetCoordFromPy */
}

/* USER CODE BEGIN Header_StartShowPicture */
/**
* @brief Function implementing the showPicture thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartShowPicture */
void StartShowPicture(void *argument)
{
  /* USER CODE BEGIN StartShowPicture */
	uint8_t count = 0;
  /* Infinite loop */
  for(;;)
  {
	  BSP_LCD_DrawRGB16Image(160, 120, 160, 120, (uint16_t *)imgBuf);
	 // osThreadFlagsWait(0x0002U, osFlagsWaitAny, osWaitForever);
	  	  if(recivedData == 1)
	  	  {
	  		  BSP_LCD_DrawRect(160+tempReciveBuf[0], 120+tempReciveBuf[3], tempReciveBuf[1] - tempReciveBuf[3], tempReciveBuf[2] - tempReciveBuf[0]);
	  		  count = count +1;
	  	  	  if (count == 3)
	  	  	  {
	  	  		  recivedData = 0;
	  		  	  count = 0;
	  	  	  }
	  	  }

  }
  /* USER CODE END StartShowPicture */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
