/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : BubbleLevel w/ FreeRTOS using ICM42688 and SSD1351
 * @author         : Moritz Emersberger
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>
#include "ssd1351.h"
#include "icm42688.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */
typedef struct delta
{
    float x;
    float y;
}DELTA;

typedef struct angles
{
    float pitch;
    float roll;
}ANGLES;

typedef struct calcdata
{
    REALACCGYR input;
    float t_roll;
    float t_pitch;
    float g_roll;
    float g_pitch;
    float roll_err;
    float pitch_err;
    float roll_err_old;
    float pitch_err_old;
    ANGLES angles;
}CALCDATA;

typedef struct coordinates
{
    int8_t x;
    int8_t y;
}COORDINATES;

typedef struct coodelta{
	DELTA delta;
	COORDINATES coo;
}COODELTA;

typedef enum
{
    LINEAR = 0,
    LOGN   = 1
}SCALE;

typedef struct anglegrid
{
    int8_t deg15r, deg30r, deg45r, deg60r, deg90r;
}ANGLEGRID;

typedef struct scalingdata
{
    SCALE scaling;
    void (*scalefunc)(float, float, DELTA*);
    ANGLEGRID angleradius;
}SCALINGDATA;

typedef enum
{
    FAST_INACCURATE = 0,
    SLOW_ACCURATE = 1
}FILTER;

typedef struct filterparam
{
    float alpha;
    float ki;
    float kp;
}FILTERPARAM;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//toggle debug/info terminal
#define DEBUG 1
#if DEBUG
#define DISPLAYTIME 1000
#else
#define DISPLAYTIME 2500
#endif


//--------- setup scanf/printf over UART2 (serial interface to PC) -----------/
#if DEBUG
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;

/* Definitions for uarttask */
osThreadId_t uarttaskHandle;
uint32_t uarttaskBuffer[ 256 ];
osStaticThreadDef_t uarttaskControlBlock;
const osThreadAttr_t uarttask_attributes = {
  .name = "uarttask",
  .cb_mem = &uarttaskControlBlock,
  .cb_size = sizeof(uarttaskControlBlock),
  .stack_mem = &uarttaskBuffer[0],
  .stack_size = sizeof(uarttaskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for spitask */
osThreadId_t spitaskHandle;
uint32_t spitaskBuffer[ 256 ];
osStaticThreadDef_t spitaskControlBlock;
const osThreadAttr_t spitask_attributes = {
  .name = "spitask",
  .cb_mem = &spitaskControlBlock,
  .cb_size = sizeof(spitaskControlBlock),
  .stack_mem = &spitaskBuffer[0],
  .stack_size = sizeof(spitaskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for i2ctask */
osThreadId_t i2ctaskHandle;
uint32_t i2ctaskBuffer[ 256 ];
osStaticThreadDef_t i2ctaskControlBlock;
const osThreadAttr_t i2ctask_attributes = {
  .name = "i2ctask",
  .cb_mem = &i2ctaskControlBlock,
  .cb_size = sizeof(i2ctaskControlBlock),
  .stack_mem = &i2ctaskBuffer[0],
  .stack_size = sizeof(i2ctaskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for fusiontask */
osThreadId_t fusiontaskHandle;
uint32_t fusiontaskBuffer[ 256 ];
osStaticThreadDef_t fusiontaskControlBlock;
const osThreadAttr_t fusiontask_attributes = {
  .name = "fusiontask",
  .cb_mem = &fusiontaskControlBlock,
  .cb_size = sizeof(fusiontaskControlBlock),
  .stack_mem = &fusiontaskBuffer[0],
  .stack_size = sizeof(fusiontaskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for converttopixelt */
osThreadId_t converttopixeltHandle;
uint32_t converttopixelBuffer[ 256 ];
osStaticThreadDef_t converttopixelControlBlock;
const osThreadAttr_t converttopixelt_attributes = {
  .name = "converttopixelt",
  .cb_mem = &converttopixelControlBlock,
  .cb_size = sizeof(converttopixelControlBlock),
  .stack_mem = &converttopixelBuffer[0],
  .stack_size = sizeof(converttopixelBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for calcdatatomonitor */
osMessageQueueId_t calcdatatomonitorHandle;
uint8_t monitordataBuffer[ 2 * sizeof( CALCDATA ) ];
osStaticMessageQDef_t monitordataControlBlock;
const osMessageQueueAttr_t calcdatatomonitor_attributes = {
  .name = "calcdatatomonitor",
  .cb_mem = &monitordataControlBlock,
  .cb_size = sizeof(monitordataControlBlock),
  .mq_mem = &monitordataBuffer,
  .mq_size = sizeof(monitordataBuffer)
};
/* Definitions for gyraccdata */
osMessageQueueId_t gyraccdataHandle;
uint8_t gyraccdataBuffer[ 2 * sizeof( REALACCGYR ) ];
osStaticMessageQDef_t gyraccdataControlBlock;
const osMessageQueueAttr_t gyraccdata_attributes = {
  .name = "gyraccdata",
  .cb_mem = &gyraccdataControlBlock,
  .cb_size = sizeof(gyraccdataControlBlock),
  .mq_mem = &gyraccdataBuffer,
  .mq_size = sizeof(gyraccdataBuffer)
};
/* Definitions for fuseddata */
osMessageQueueId_t fuseddataHandle;
uint8_t fuseddataBuffer[ 2 * sizeof( ANGLES ) ];
osStaticMessageQDef_t fuseddataControlBlock;
const osMessageQueueAttr_t fuseddata_attributes = {
  .name = "fuseddata",
  .cb_mem = &fuseddataControlBlock,
  .cb_size = sizeof(fuseddataControlBlock),
  .mq_mem = &fuseddataBuffer,
  .mq_size = sizeof(fuseddataBuffer)
};
/* Definitions for coodata */
osMessageQueueId_t coodataHandle;
uint8_t coodataBuffer[ 1 * sizeof( COORDINATES ) ];
osStaticMessageQDef_t coodataControlBlock;
const osMessageQueueAttr_t coodata_attributes = {
  .name = "coodata",
  .cb_mem = &coodataControlBlock,
  .cb_size = sizeof(coodataControlBlock),
  .mq_mem = &coodataBuffer,
  .mq_size = sizeof(coodataBuffer)
};
/* Definitions for pixeldatatomonitor */
osMessageQueueId_t pixeldatatomonitorHandle;
uint8_t pixeldatatomonitorBuffer[ 1 * sizeof( COODELTA ) ];
osStaticMessageQDef_t pixeldatatomonitorControlBlock;
const osMessageQueueAttr_t pixeldatatomonitor_attributes = {
  .name = "pixeldatatomonitor",
  .cb_mem = &pixeldatatomonitorControlBlock,
  .cb_size = sizeof(pixeldatatomonitorControlBlock),
  .mq_mem = &pixeldatatomonitorBuffer,
  .mq_size = sizeof(pixeldatatomonitorBuffer)
};
/* Definitions for fuseddataavailable */
osSemaphoreId_t fuseddataavailableHandle;
const osSemaphoreAttr_t fuseddataavailable_attributes = {
  .name = "fuseddataavailable"
};
/* Definitions for cooavailable */
osSemaphoreId_t cooavailableHandle;
const osSemaphoreAttr_t cooavailable_attributes = {
  .name = "cooavailable"
};
/* Definitions for rawdataavailable */
osSemaphoreId_t rawdataavailableHandle;
const osSemaphoreAttr_t rawdataavailable_attributes = {
  .name = "rawdataavailable"
};
/* Definitions for monitorcalcavailable */
osSemaphoreId_t monitorcalcavailableHandle;
const osSemaphoreAttr_t monitorcalcavailable_attributes = {
  .name = "monitorcalcavailable"
};
/* USER CODE BEGIN PV */
static const ANGLEGRID angleraddata[2] = {
        { 8,15,23,31,46},
        {11,21,28,35,46}
};

static const FILTERPARAM filterparameter[2] = {
    {0.7,  1, 1},
    {0.7,0.1,25}
};

CALDATA cal;
SCALINGDATA griddata;
FILTERPARAM filterdata;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
void startuarttask(void *argument);
void startspitask(void *argument);
void starti2ctask(void *argument);
void startfusiontask(void *argument);
void startconverttopixeltask(void *argument);

/* USER CODE BEGIN PFP */
uint8_t get_adc_raw();
void ics_init(CALDATA*);
void scale_linear(float, float, DELTA*);
void scale_ln(float, float, DELTA*);
static const void (*fptr[2])(float,float,DELTA*)={&scale_linear, &scale_ln};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if DEBUG
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

GETCHAR_PROTOTYPE
{
    uint8_t ch = 0;

    /* Clear the Overrun flag just before receiving the first character */
    __HAL_UART_CLEAR_OREFLAG(&huart2);

    /* Wait for reception of a character on the USART RX line and echo this
     * character on console */
    HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#endif
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
  setvbuf(stdin, NULL, _IONBF, 0);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  //initialise screen, show startup-info, select scaling, initialise 6DOF
  ics_init(&cal);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of fuseddataavailable */
  fuseddataavailableHandle = osSemaphoreNew(1, 0, &fuseddataavailable_attributes);

  /* creation of cooavailable */
  cooavailableHandle = osSemaphoreNew(1, 0, &cooavailable_attributes);

  /* creation of rawdataavailable */
  rawdataavailableHandle = osSemaphoreNew(1, 0, &rawdataavailable_attributes);
#if DEBUG
  /* creation of monitorcalcavailable */
  monitorcalcavailableHandle = osSemaphoreNew(1, 0, &monitorcalcavailable_attributes);
#endif
  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
#if DEBUG
  /* creation of calcdatatomonitor */
  calcdatatomonitorHandle = osMessageQueueNew (2, sizeof(CALCDATA), &calcdatatomonitor_attributes);
#endif
  /* creation of gyraccdata */
  gyraccdataHandle = osMessageQueueNew (2, sizeof(REALACCGYR), &gyraccdata_attributes);

  /* creation of fuseddata */
  fuseddataHandle = osMessageQueueNew (2, sizeof(ANGLES), &fuseddata_attributes);

  /* creation of coodata */
  coodataHandle = osMessageQueueNew (1, sizeof(COORDINATES), &coodata_attributes);
#if DEBUG
  /* creation of pixeldatatomonitor */
  pixeldatatomonitorHandle = osMessageQueueNew (1, sizeof(COODELTA), &pixeldatatomonitor_attributes);
#endif
  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
#if DEBUG
  /* creation of uarttask */
  uarttaskHandle = osThreadNew(startuarttask, NULL, &uarttask_attributes);
#endif
  /* creation of spitask */
  spitaskHandle = osThreadNew(startspitask, (void*) &griddata.angleradius, &spitask_attributes);

  /* creation of i2ctask */
  i2ctaskHandle = osThreadNew(starti2ctask, (void*) &cal, &i2ctask_attributes);

  /* creation of fusiontask */
  fusiontaskHandle = osThreadNew(startfusiontask, (void*) &filterdata, &fusiontask_attributes);

  /* creation of converttopixelt */
  converttopixeltHandle = osThreadNew(startconverttopixeltask, (void*) griddata.scalefunc, &converttopixelt_attributes);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
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
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0050040C;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /** I2C Fast mode Plus enable
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_dis_GPIO_Port, RST_dis_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RW_dis_Pin|EN_dis_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_gyr_GPIO_Port, CS_gyr_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_dis_GPIO_Port, CS_dis_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DC_dis_GPIO_Port, DC_dis_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RST_dis_Pin */
  GPIO_InitStruct.Pin = RST_dis_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_dis_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RW_dis_Pin EN_dis_Pin */
  GPIO_InitStruct.Pin = RW_dis_Pin|EN_dis_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_gyr_Pin */
  GPIO_InitStruct.Pin = INT_gyr_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_gyr_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_gyr_Pin */
  GPIO_InitStruct.Pin = CS_gyr_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_gyr_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_dis_Pin DC_dis_Pin */
  GPIO_InitStruct.Pin = CS_dis_Pin|DC_dis_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */


/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief reads analog input
 * @retval 8bit unsigned adc value
 */
uint8_t get_adc_raw()
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    uint8_t adc_value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return adc_value;
}

/**
 * @brief scales pitch and roll linearly for 96px screen
 * @param pitch: sensor fused pitch from accel- and gyro-data
 * @param pitch: sensor fused roll  from accel- and gyro-data
 * @param delta: pointer to struct of two floats for result
 * @retval None
 */
void scale_linear(float pitch, float roll, DELTA* delta)
{
	//PI/2 ~= 1.5707963
    delta->x = (fabs(pitch)/1.5707963)*46;
    delta->y = (fabs(roll )/1.5707963)*46;
}

/**
 * @brief scales pitch and roll logarithmically for 96px screen
 * @param pitch: sensor fused pitch from accel- and gyro-data
 * @param pitch: sensor fused roll  from accel- and gyro-data
 * @param delta: pointer to struct of two floats for result
 * @retval None
 */
void scale_ln(float pitch, float roll, DELTA* delta)
{
	//log nat((PI/2)+1) ~= 0.9442157
    delta->x = ((logf(fabs(pitch)+1))/ 0.9442157)*46;
    delta->y = ((logf(fabs( roll)+1))/ 0.9442157)*46;
}

/**
 * @brief initialises display and 6DOF, configures scaling with userinput
 * @param cal: pointer to sensor-calibration results structure
 * @retval None
 */
void ics_init(CALDATA* cal)
{
    //reset display
    SSD1351_SetPin(RESET_PORT, RESET_PIN);
    SSD1351_ClearPin(RESET_PORT, RESET_PIN);
    SSD1351_SetPin(RESET_PORT, RESET_PIN);
    HAL_GPIO_WritePin(GPIOA, EN_dis_Pin, GPIO_PIN_SET);

    //configure and check display-driver registers
    SSD1351_init();

    //startup screen
    SSD1351_fill65k(COLOUR_BLACK);
    SSD1351_set_cursor(0,95);
    SSD1351_printf(SSD1351_get_rgb(255,  0,  0), small_font, "BubbleLevelw/\nFreeRTOS\n");
    SSD1351_printf(SSD1351_get_rgb(255,142,  0), small_font, "6DOF:ICM42688\n");
    SSD1351_printf(SSD1351_get_rgb(255,255,  0), small_font, "OLED:PSP27801\n");
    SSD1351_printf(SSD1351_get_rgb(  0,142,  0), small_font, "OLED-drv:\nSSD1351\n" );
    SSD1351_printf(SSD1351_get_rgb( 64,  0,152), small_font, "application\nby:");
    SSD1351_printf(SSD1351_get_rgb(142,  0,142), small_font, "Moritz\nEmersberger" );
    SSD1351_update();
    HAL_Delay(DISPLAYTIME);
#if DEBUG
    printf("\r\n\nBubbleLevel with FreeRTOS\n\r");
    printf("6DOF: ICM42688\n\r");
    printf("OLED: PSP27801\n\r");
    printf("OLED-drv: SSD1351\n\r" );
    printf("application by:\n\r");
    printf("Moritz Emersberger\n\n\r");
#endif

    //scale-selection + screen
    char text[2][26] = {"linear\n", "LOGnat\n"};
    char scale_text[26];
    int8_t selection = 0, selection_previous = 0;
    uint8_t adc_input=0;
    int8_t i;
    for (i = 5; i > 0; i--) {

        //get input from potentiometer and choose scaling
        adc_input = get_adc_raw();
        if (adc_input < 128) {
            selection = LOGN;
        } else {
            selection = LINEAR;
        }
        snprintf(scale_text, 26, "%s", text[selection]);

        // draw difficulty selection screen
        SSD1351_fill65k(COLOUR_BLACK);
        SSD1351_set_cursor(0,95);
        SSD1351_printf(COLOUR_WHITE, med_font, "scale-\nselect\nw/ADC:\n");
        SSD1351_printf(COLOUR_GREY, med_font, scale_text);
        SSD1351_update();
#if DEBUG
        printf("RAW-ADC input for scale-select: %d\r\n", adc_input);
#endif
        // reset counter if difficulty user input changed
        if (selection_previous != selection){ i = 5;}
        selection_previous = selection;
        HAL_Delay(500);
    }
    //assign scaling from enum to griddata for spi- and converttopixeltasks
    griddata.scaling = selection;
    griddata.angleradius = angleraddata[griddata.scaling];
    griddata.scalefunc   = fptr[griddata.scaling];
#if DEBUG
    printf("\nscale selected: %s\r\n", scale_text);
#endif
    //filter-settings selection
    snprintf(text[0],26,"%s","alpha:0.7\nki:1\nkp:1");
    snprintf(text[1],26,"%s","alpha:0.7\nki:0.1\nkp:25");
    char filter_text[26];
    selection = 0;
    selection_previous = 0;
    adc_input=0;
    for (i = 5; i > 0; i--) {

        //get input from potentiometer and choose scaling
        adc_input = get_adc_raw();
        if (adc_input < 128) {
            selection = 1;
        } else {
            selection = 0;
        }
        snprintf(filter_text, 26, "%s", text[selection]);

        // draw difficulty selection screen
        SSD1351_fill65k(COLOUR_BLACK);
        SSD1351_set_cursor(0,95);
        SSD1351_printf(COLOUR_WHITE, med_font, "filter-\nselect\nw/ADC:\n");
        SSD1351_printf(COLOUR_GREY, small_font, filter_text);
        SSD1351_update();
#if DEBUG
        printf("RAW-ADC input for filter-select: %d\r\n", adc_input);
#endif
        // reset counter if difficulty user input changed
        if (selection_previous != selection){ i = 5;}
        selection_previous = selection;
        HAL_Delay(500);
    }
    filterdata = filterparameter[selection];
#if DEBUG
    printf("\nfilter values selected: alpha=%f ki=%f kp=%f\r\n", filterdata.alpha, filterdata.ki, filterdata.kp);
#endif

    //legend screen
    SSD1351_fill65k(COLOUR_BLACK);
    SSD1351_set_cursor(0,95);
    SSD1351_printf(SSD1351_get_rgb(142,  0,142), med_font, scale_text);
    SSD1351_printf(SSD1351_get_rgb(142,  0,142), small_font, filter_text);
    SSD1351_printf(COLOUR_RED, small_font, "\n15deg | PI/12\n");
    SSD1351_printf(COLOUR_BLUE, small_font, "30deg | PI/6\n");
    SSD1351_printf(COLOUR_GREEN, small_font, "45deg | PI/4\n");
    SSD1351_printf(COLOUR_YELLOW, small_font, "60deg | PI/3\n");
    SSD1351_printf(COLOUR_PURPLE, small_font, "~90deg|~PI/2\n");
    SSD1351_update();

    //place on level surface
#if DEBUG
#else
    SSD1351_fill65k(COLOUR_BLACK);
    SSD1351_set_cursor(0,95);
    SSD1351_printf(COLOUR_WHITE, small_font, "Please place\n device on\nlevel surface\n\n Calibration\nwill start in 5 seconds!");
    HAL_Delay(5000);
    //HAL_Delay(1000);
    SSD1351_update();
    //clear and write seconds for countdown
    int8_t x = SSD1351_get_cursor_x()-70;
    for(int8_t i=4; i>0; i--){
        HAL_Delay(1000);
        SSD1351_set_cursor(x ,SSD1351_get_cursor_y());
        SSD1351_draw_filled_rect(SSD1351_get_cursor_x(), SSD1351_get_cursor_y()-9, 7, 10, COLOUR_BLACK);
        SSD1351_printf(COLOUR_WHITE, small_font, "%i", i);
        SSD1351_update();
    }
#endif
    //configure and check 6DOF registers
    ICM42688_init();
    //calibration screen, continues in ICEM42688_calibrate()
    SSD1351_fill65k(COLOUR_BLACK);
    SSD1351_set_cursor(0,95);
    SSD1351_printf(COLOUR_WHITE, small_font, "Calibrating..\n.............\n\nDuration: 10s\n(1000 samples\n10ms/sample)\n");
    SSD1351_printf(COLOUR_WHITE, small_font, "Time left:\n10 seconds");
    HAL_Delay(1000);
    SSD1351_update();
    ICM42688_calibrate(cal);
    HAL_Delay(50);

    //initialisation of ICs finished, FreeRTOS will be started
    SSD1351_fill65k(COLOUR_BLACK);
    SSD1351_set_cursor(0,95);
    SSD1351_printf(COLOUR_GREEN, med_font, "Bubble\nLevel w/\nFreeRTOS\nstarts\n");
    SSD1351_printf(COLOUR_RED, med_font, "NOW");
    SSD1351_update();
#if DEBUG
    printf("\nBubbleLevel with FreeRTOS starts now!\r\n\n");
#endif
    HAL_Delay(DISPLAYTIME);
}

/**
 * @brief reads 1000times from 6DOF-sensors and saves average as calibration values
 *        more sophisticated calibration routine w.r.t. earth's gravity could be implemented in future
 * @param cal: pointer to sensor-calibration results structure
 * @retval None
 */
void ICM42688_calibrate(CALDATA* cal)
{
    DATASET data;
    int8_t j = 9;
    int8_t x = SSD1351_get_cursor_x()-70;

    for(uint16_t i=0; i<1000; i++)
    {
        //get sensor readings and save to double
        ICM42688_read_all_sensors(&data.raw, &data.conv);
        cal->gyrox += data.conv.gyrox;
        cal->gyroy += data.conv.gyroy;
        cal->accx += data.conv.accx;
        cal->accy += data.conv.accy;
        cal->accz += data.conv.accz;

        //update displays countdown
        if(i%100==0 && i>0){
            SSD1351_set_cursor(x ,SSD1351_get_cursor_y());
            SSD1351_draw_filled_rect(x, SSD1351_get_cursor_y()-9, 14, 10, COLOUR_BLACK);
            SSD1351_printf(COLOUR_WHITE, small_font, " %i", j);
            SSD1351_update();
            j--;
        }
        HAL_Delay(10);
    }

    //average saved data

    //16.4 for fs_sel0, 262 for fs_sel=4, 131 for fs_sel=3, 65.5 for fs_sel=2, 2097.2 for fs_sel=7
    cal->gyrox/= 16400;//(1000*131 bzw. 16.4);
    cal->gyroy/= 16400;//(1000*131 bzw. 16.4);

    //16384 for fs_sel=3, 8192 for fs_sel=2
    cal->accx/= 16384000;//(1000*16384);
    cal->accy/= 16384000;//(1000*16384);
    cal->accz/= 16384000;//(1000*16384);
#if DEBUG
    printf("\rCalibration finished, offset-values:\n\r");
    printf("\tgyro-x: %10f gyro-y: %10f\r\n", cal->gyrox, cal->gyroy);
    printf("\tacc-x : %10f acc-y: %10f acc-z: %10f\r\n", cal->accx, cal->accy, cal->accz);
#endif
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_startuarttask */
/**
 * @brief  switchable via DEFINE, prints DEBUG/INFO data to connected Terminal
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_startuarttask */
void startuarttask(void *argument)
{
  /* USER CODE BEGIN 5 */

    osStatus_t state, state2;
    CALCDATA calc;
    COODELTA cd;
    //osThreadState_t threadstate;
    bool calcavailable=false;
    bool pixelavailable=false;
    /* Infinite loop */

    for(;;)
    {
        state= osMessageQueueGet(calcdatatomonitorHandle, &calc, 0U, 0);
        //state= osMessageQueueGet(monitordataHandle, &degtest, 0U, 10);
        if(osOK == state ){
        	calcavailable=true;

        	//printf("gettingdebugcalc ok");

        }else{
            //printf("\tgetting debugcalc, state: %i\r\n", state);
        }
        state2= osMessageQueueGet(pixeldatatomonitorHandle, &cd, 0U, 0);
        if(osOK == state2 ){
        	pixelavailable=true;

        	//printf("gettingdebugcoo ok");

        }else{
        	//printf("\tgetting debugcoo, state: %i\r\n", state);
        }

        if((calcavailable==true) && (pixelavailable==true)){

        	printf("**********************************BubbleLevel DebugData**********************************\n\r");
        	printf("gyro  raw-data gyrx gyry [rad/s]           :  %10f %10f \r\n", 			calc.input.gyrx, calc.input.gyry);
        	printf("accel raw-data accx accy accz [m/s*s]      :  %10f %10f %10f\r\n", 		calc.input.accx, calc.input.accy, calc.input.accz );
        	printf("t_roll t_pitch g_roll g_pitch [rad]        :  %10f %10f %10f %10f\r\n", calc.t_roll, calc.t_pitch, calc.g_roll, calc.g_pitch);
        	printf("error: roll pitch roll_old pitch_old [rad] :  %10f %10f %10f %10f\r\n", calc.roll_err, calc.pitch_err, calc.roll_err_old, calc.pitch_err_old );
        	printf("roll pitch [rad]                           :  %10f %10f\r\n", calc.angles.roll, calc.angles.pitch);
        	printf("delta_x delta_y [px in float]              :  %10f %10f\r\n", cd.delta.x, cd.delta.y );
        	printf("coordinates x y [px in int8_t]             :   %d         %d\r\n", cd.coo.x, cd.coo.y );
        	printf("*****************************************************************************************\n\n\r");

        	calcavailable = false;
        	pixelavailable = false;
        }
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startspitask */
/**
 * @brief updates display with current roll/pitch-values i.e. corresponding pixel-positions from converttopixeltask
 * @param argument: scale-dependent grid-drawing values
 * @retval None
 */
/* USER CODE END Header_startspitask */
void startspitask(void *argument)
{
  /* USER CODE BEGIN startspitask */
    osStatus_t state;
    COORDINATES coo;
    ANGLEGRID* ptr = argument;
    ANGLEGRID griddata;
    griddata.deg15r = ptr->deg15r;
    griddata.deg30r = ptr->deg30r;
    griddata.deg45r = ptr->deg45r;
    griddata.deg60r = ptr->deg60r;
    griddata.deg90r = ptr->deg90r;
    coo.x=46; coo.y=46;
    /* Infinite loop */
    for(;;)
    {
        //info from converttopixeltask, if new pixel-data is available
        if(osSemaphoreAcquire(cooavailableHandle,0)==osOK){
            SSD1351_fill65k(COLOUR_BLACK);

            //draw cross
            SSD1351_draw_line(47,2,47,92,COLOUR_WHITE);
            SSD1351_draw_line(48,2,48,92,COLOUR_WHITE);
            SSD1351_draw_line(2,47,92,47,COLOUR_WHITE);
            SSD1351_draw_line(2,48,92,48,COLOUR_WHITE);

            //draw grid
            SSD1351_draw_rect(48-griddata.deg15r,48-griddata.deg15r,griddata.deg15r*2,griddata.deg15r*2,COLOUR_RED); //15 deg
            SSD1351_draw_rect(48-griddata.deg30r,48-griddata.deg30r,griddata.deg30r*2,griddata.deg30r*2,COLOUR_BLUE);//30 deg
            SSD1351_draw_rect(48-griddata.deg45r,48-griddata.deg45r,griddata.deg45r*2,griddata.deg45r*2,COLOUR_GREEN);//45 deg
            SSD1351_draw_rect(48-griddata.deg60r,48-griddata.deg60r,griddata.deg60r*2,griddata.deg60r*2,COLOUR_YELLOW);//60 deg
            SSD1351_draw_rect(48-griddata.deg90r,48-griddata.deg90r,griddata.deg90r*2,griddata.deg90r*2,COLOUR_PURPLE); //90 deg

            //get new pixel data
            state= osMessageQueueGet(coodataHandle, &coo, 0U, osWaitForever);
            if(osOK != state ){
                printf("\tgetting-coo: %i\r\n", state);
            }

            //draw bubble corresponding to pitch/roll
            SSD1351_draw_filled_circle(coo.x,coo.y,2,COLOUR_AQUA);
            SSD1351_update();
        }
    }
  /* USER CODE END startspitask */
}

/* USER CODE BEGIN Header_starti2ctask */
/**
 * @brief gets raw accelorometer and gyroscope data from 6DOF-IC via I2C,
 *        converts and sends these to fusiontask
 * @param argument: calibration values from ICM42688_calibrate();
 * @retval None
 */
/* USER CODE END Header_starti2ctask */
void starti2ctask(void *argument)
{
  /* USER CODE BEGIN starti2ctask */
    DATASET data;
    REALACCGYR output;
    CALDATA* ptr = argument;
    CALDATA cali;
    cali.accx = ptr->accx;
    cali.accy = ptr->accy;
    cali.accz = ptr->accz;
    cali.gyrox = ptr->gyrox;
    cali.gyroy = ptr->gyroy;
    osStatus_t state;
    //osThreadState_t threadstate;
    /* Infinite loop */
    for(;;)
    {
        //reads all sensors and converts values to 2s-complement
        ICM42688_read_all_sensors(&data.raw, &data.conv);
        //subtract calibration values (not used for z-axis, since reference value must not be 0 (earth's gravity))
        output.accx = (float)data.conv.accx/16384 - cali.accx;
        output.accy = (float)data.conv.accy/16384 - cali.accy;
        output.accz = (float)data.conv.accz/16384;
        output.gyrx = ((float)data.conv.gyrox/16.4 - cali.gyrox)*0.0174532925;
        output.gyry = ((float)data.conv.gyroy/16.4 - cali.gyroy)*0.0174532925;

        //send data to fusiontask and notify for availability
        osMessageQueuePut(gyraccdataHandle, &output, 0U, 0);
        state= osSemaphoreRelease(rawdataavailableHandle);
        if(osOK != state ){
            printf("\tputting, rawdataavailstate:%i\r\n", state);
        }
        osDelay(10);
    }
  /* USER CODE END starti2ctask */
}

/* USER CODE BEGIN Header_startfusiontask */
/**
 * @brief implements sensor fusion of accel- and gyro via complementary filter with data from i2ctask,
 *        sends fused-data to converttopixeltask
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startfusiontask */
void startfusiontask(void *argument)
{
  /* USER CODE BEGIN startfusiontask */

    osStatus_t state, state2;
    CALCDATA calc;
    calc.angles.roll  = 0;
    calc.angles.pitch = 0;
    calc.pitch_err = 0;
    calc.pitch_err_old = 0;
    calc.roll_err = 0;
    calc.roll_err_old = 0;
    FILTERPARAM* ptr = argument;
    FILTERPARAM filt;
    filt.alpha = ptr->alpha;
    filt.ki = ptr->ki;
    filt.kp = ptr->kp;
#if DEBUG
    osStatus_t state3;
    int8_t i = 0;
#endif
    osSemaphoreRelease(fuseddataavailableHandle);

    /* Infinite loop */
    for(;;)
    {
        //notification from i2ctask, if raw sensor-data is available
        if(osSemaphoreAcquire(rawdataavailableHandle, osWaitForever) == osOK){
            state= osMessageQueueGet(gyraccdataHandle, &calc.input, 0U, 10);
            if(osOK == state ){
                //calculate angle from linear acceleration
                if(calc.input.accz>0.0){
                    //for device upwards
                    calc.t_roll  = atan2f(calc.input.accy, calc.input.accz);
                    //calc.t_roll   = atan2f(calc.input.accy, sqrtf(powf(calc.input.accy,2)+powf(calc.input.accz,2)));
                    calc.t_pitch = atan2f(calc.input.accx, calc.input.accz);
                    //calc.t_pitch  = atan2f(calc.input.accx, sqrtf(powf(calc.input.accx,2)+powf(calc.input.accz,2)));
                }else
                {
                    //device downwards
                    calc.t_roll  = atan2f(calc.input.accy, -calc.input.accz);
                    calc.t_pitch = atan2f(calc.input.accx, -calc.input.accz);
                }

                //get error between last fused-angles and - via linear accel. calculated - angles
                calc.roll_err  = calc.t_roll  - calc.angles.roll;
                calc.pitch_err = calc.t_pitch - calc.angles.pitch;

                //integrate from last fused angle over error-compensated
                //(via PI controller on error between fused-angles and angles from linear accel.) angular accel (from gyro)
                calc.g_roll  = (calc.input.gyrx - (calc.roll_err_old  + (filt.ki*0.01 + filt.kp) * calc.roll_err ) ) * 0.01 + calc.angles.roll;
                calc.g_pitch = (calc.input.gyry - (calc.pitch_err_old + (filt.ki*0.01 + filt.kp) * calc.pitch_err) ) * 0.01 + calc.angles.pitch;

                //complementary filter, fuses angles from linear and angular acceleration with weight
                calc.angles.roll  = calc.g_roll  * filt.alpha + calc.t_roll  * (1-filt.alpha);
                calc.angles.pitch = calc.g_pitch * filt.alpha + calc.t_pitch * (1-filt.alpha);

                //save old error between last fused-angles and - via linear accel. calculated - angles
                calc.roll_err_old  = calc.roll_err;
                calc.pitch_err_old = calc.pitch_err;

                //printf("groll, gpitch, ang_rol, ang_pit %f %f %f %f \r\n",calc.g_roll, calc.g_pitch, calc.angles.roll, calc.angles.pitch);
                //printf("x y z gyrx gyry: %f %f %f %f %f\r\n", calc.input.accx, calc.input.accy, calc.input.accz, calc.input.gyrx, calc.input.gyry);

                //check, if last fused data was taken from queue from converttopixeltask
                if(osSemaphoreAcquire(fuseddataavailableHandle,0)== osOK){
                    state2 = osMessageQueuePut(fuseddataHandle, &calc.angles, 0U, 0);
                    if(osOK != state2 ){
                        printf("angles put state:%i\r\n", state2);
                    }
#if DEBUG
                    //if(i==25){
					if(i==50){
						state3 = osMessageQueuePut(calcdatatomonitorHandle, &calc, 0U, 0);

						if(state3 != osOK){
							printf("debug calc putting: %i\r\n", state3);
						}
						if(osSemaphoreAcquire(monitorcalcavailableHandle,0)!=osOK){printf("monitorsem not ok\r\n");}
						i=0;
					}else{

						i++;
					}
#endif
                }
            }else{
                printf("\tgetting, gyrdatstate: %i\r\n", state);
            }
        }
    }
  /* USER CODE END startfusiontask */
}

/* USER CODE BEGIN Header_startconverttopixeltask */
/**
 * @brief maps angle in radiant (from fusiontask) to pixel-coordinates with selected scaling,
 *        sends coordinates in pixel to spitask
 * @param argument: function pointer to scaling function (linear or logarithmic)
 * @retval None
 */
/* USER CODE END Header_startconverttopixeltask */
void startconverttopixeltask(void *argument)
{
  /* USER CODE BEGIN startconverttopixeltask */
    osStatus_t state, state2;
    ANGLES angles;
    void (*scaleptr)(float, float, DELTA*);
    scaleptr = argument;
    COODELTA cd;
    cd.coo.x=0; cd.coo.y=0;
#if DEBUG
    osStatus_t state3;
#endif
    /* Infinite loop */
    for(;;)
    {
    	//get new fused-data from fusiontask
    	state= osMessageQueueGet(fuseddataHandle, &angles, 0U, osWaitForever);
    	if(osOK == state){
    		//call chosen scaling function (linear or logarithmic)
    		scaleptr(angles.pitch, angles.roll, &cd.delta);

    		//quadrant-specific calculation of pixel-coordinates
    		if(angles.roll < 0.0){
    			cd.coo.y = round(47.5 - cd.delta.y);
    			if(cd.coo.y < 1){cd.coo.y = 1;}
    			if(angles.pitch < 0.0){
    				cd.coo.x = round(47.5 - cd.delta.x);
    				if(cd.coo.x < 1){cd.coo.x = 1;}
    			}else
    			{
    				cd.coo.x = round(47.5 + cd.delta.x);
    				if(cd.coo.x > 95){cd.coo.x = 95;}
    			}

    		}else
    		{
    			cd.coo.y = round(47.5 + cd.delta.y);
    			if(cd.coo.y > 95){cd.coo.y = 95;}
    			if(angles.pitch < 0.0){
    				cd.coo.x = round(47.5 - cd.delta.x);
    				if(cd.coo.x < 1){cd.coo.x = 1;}
    			}else
    			{
    				cd.coo.x = round(47.5 + cd.delta.x);
    				if(cd.coo.x > 95){cd.coo.x = 95;}
    			}
    		}

    		//send pixel-coordinates to spitask
    		state2 = osMessageQueuePut(coodataHandle, &cd.coo, 0U, 0);
    		if(osOK == state2){
    			osSemaphoreRelease(cooavailableHandle);
    		}
#if DEBUG
    		if(osSemaphoreRelease(monitorcalcavailableHandle)==osOK){
    			state3 = osMessageQueuePut(pixeldatatomonitorHandle, &cd, 0U, 0);

    			if(state3 != osOK){
    				printf("debug pixel putting: %i\r\n", state3);
    			}
    		}
#endif
    		//notify fusiontask, that new fused data can be put into queue
    		osSemaphoreRelease(fuseddataavailableHandle);
    	}else
    	{
    		printf("angles, getting: %i\r\n", state);
    	}
    }
  /* USER CODE END startconverttopixeltask */
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
    printf("\rError in File: %s, Line: %d\n\r",__FILE__, __LINE__);
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
