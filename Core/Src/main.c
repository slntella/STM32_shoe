/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : This program is used for the BRIDGE PCB. It is reading
  * 				  the force sensor values, locates the high pressure spots
  * 				  and offloads automatically the two highest pressure spots if activated
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUMBER_OF_SENSORS  				 31
#define MAX                				 100

/* Configure MAX77874 Buck Converter */
#define MAX77874_BUCK_CONV_SLAVE_ADDRESS 0x61
#define MAX77874_BUCK_CONV_READ_ADDRESS  0xC1
#define MAX77874_BUCK_CONV_WRITE_ADDRESS 0xC2

/* Configure DA9070 PMIC */
//#define DA9070_PMIC_SLAVE_ADDRESS       0x68
//#define DA9070_PMIC_READ_ADDRESS        0xD1
//#define DA9070_PMIC_WRITE_ADDRESS       0xD0

#define START_BIT						 15615.0
#define STOP_BIT						 15700.0
/* Value of analog reference voltage (Vref+), connected to analog voltage   */
/* supply Vdda (unit: mV).                                                  */
#define VDDA_APPLI                       (3300UL)

/* Init variable out of expected ADC conversion data range */
#define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(ADC1, LL_ADC_RESOLUTION_12B) + 1)
/* RX buffer size -1*/
#define UART_RX_IDX_MAX 127

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc4;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
DMA_NodeTypeDef Node_GPDMA1_Channel0;
DMA_QListTypeDef List_GPDMA1_Channel0;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

/* USER CODE BEGIN PV */
uint8_t  		data_written_spi[5] = {0xFF,0xFF,0xFF,0xFF,0xFF};
//uint8_t  		data_written_spi[5] = {0,0,0,0,0};
uint32_t 		value_adc[NUMBER_OF_SENSORS]; // force sensor values read by ADC in every measurement
uint64_t        modules_state_uint = 0xFFFFFFFF;
char			uart_rx_buf[UART_RX_IDX_MAX+1]; //uart rx buffer

static volatile uint8_t  autocontrol_timeout = 0;
static uint8_t  autocontrol = 0; //if true, triggers auto control, if not does manual control

static float 	pressure_map[NUMBER_OF_SENSORS];
static float    average_pressure_map[NUMBER_OF_SENSORS];
static float    previous_pressure_map[NUMBER_OF_SENSORS];
static int8_t   modules_state[NUMBER_OF_SENSORS];
static float 	max_1 				   = 0.0;
static float 	max_2 				   = 0.0;
static int 		counter_average        = 0;  // nb of TIMER4 cycles
static int		index_max_1			   = 0;
static int 		index_max_2			   = 0;
static int      index_max_1_previous   = 0;
static int      index_max_2_previous   = 0;
static int 		counter_modules_off    = 0;
const int 		select_pins_MUX_1[5]   = {A0_MUX_Pin, A1_MUX_Pin, A2_MUX_Pin, A3_MUX_Pin, A4_MUX_Pin};
const int 		select_pins_MUX_2[3]   = {MA0_Pin, MA1_Pin, MA2_Pin};

const uint16_t valve2SPIbit[] = {
		[0] = 0,
		[1] = 9,
		[2] = 20,
		[3] = 30,
		[4] = 40,
		[5] = 10,
		[6] = 21,
		[7] = 32,
		[8] = 31,
		[9] = 25,
		[10] = 11,
		[11] = 23,
		[12] = 22,
		[13] = 17,
		[14] = 26,
		[15] = 35, //ou 12
		[16] = 24,
		[17] = 18,
		[18] = 28,
		[19] = 27,
		[20] = 33,
		[21] = 13,
		[22] = 19,
		[23] = 29,
		[24] = 14,
		[25] = 2,
		[26] = 3,
		[27] = 15,
		[28] = 16,
		[29] = 4,
		[30] = 1,
		[31] = 5
};
const uint16_t Mes2Sensor[] = {
		[1] = 22,
		[2] = 17,
		[3] = 16,
		[4] = 5,
		[5] = 23,
		[6] = 18,
		[7] = 15,
		[8] = 9,
		[9] = 6,
		[10] = 24,
		[11] = 21,
		[12] = 14,
		[13] = 10,
		[14] = 32,
		[15] = 25,
		[16] = 20,
		[17] = 13,
		[18] = 11,
		[19] = 8,
		[20] = 26,
		[21] = 19,
		[22] = 12,
		[23] = 7,
		[24] = 32,
		[25] = 32,
		[26] = 4,
		[27] = 32,
		[28] = 1,
		[29] = 3,
		[30] = 32,
		[31] = 2
};

/*
const uint16_t Mes2Sensor[] = {
		[0] = 0,
		[1] = 27,
		[2] = 22,
		[3] = 21,
		[4] = 9,
		[5] = 28,
		[6] = 23,
		[7] = 20,
		[8] = 14,
		[9] = 10,
		[10] = 29,
		[11] = 24,
		[12] = 19,
		[13] = 15,
		[14] = 13,
		[15] = 30,
		[16] = 25,
		[17] = 18,
		[18] = 16,
		[19] = 12,
		[20] = 31,
		[21] = 24,
		[22] = 17,
		[23] = 11,
		[24] = 1,
		[25] = 4,
		[26] = 8,
		[27] = 2,
		[28] = 5,
		[29] = 7,
		[30] = 3,
		[31] = 6
};
*/
static float bias[NUMBER_OF_SENSORS] = {
		 [1] = -270,
		 [2] = -175,
		 [3] = -195,
		 [4] = -185,
		 [5] = -175,
		 [6] = -185,
		 [7] = -190,
		 [8] = -190,
		 [9] = -152,
		 [10] = -172,
		 [11] = -190,
		 [12] = -215,
		 [13] = -215,
		 [14] = -190,
		 [15] = -20,
		 [16] = -190,
		 [17] = -195,
		 [18] = -195,
		 [19] = -210,
		 [20] = -250,
		 [21] = -165,
		 [22] = -190,
		 [23] = -130
};
static float slope[NUMBER_OF_SENSORS] = {
		 [1] = 160,
		 [2] = 120,
		 [3] = 120,
		 [4] = 120,
		 [5] = 120,
		 [6] = 120,
		 [7] = 120,
		 [8] = 120,
		 [9] = 115,
		 [10] = 120,
		 [11] = 120,
		 [12] = 135,
		 [13] = 120,
		 [14] = 120,
		 [15] = 51,
		 [16] = 120,
		 [17] = 120,
		 [18] = 120,
		 [19] = 120,
		 [20] = 140,
		 [21] = 120,
		 [22] = 120,
		 [23] = 100
};


 /* Variable to report status of ADC group regular unitary conversion          */
 /*  0: ADC group regular unitary conversion is not completed                  */
 /*  1: ADC group regular unitary conversion is completed                      */
 /*  2: ADC group regular unitary conversion has not been started yet          */
 /*     (initial state)                                                        */
 __IO uint8_t ubAdcGrpRegularUnitaryConvStatus = 2; /* Variable set into ADC interruption callback */

 /* Variables for ADC conversion data */
 __IO uint16_t uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE; /* ADC group regular conversion data */

 /* Variables for ADC conversion data computation to physical values */
 uint16_t uhADCxConvertedData_Voltage_mVolt = 0;  /* Value of voltage calculated from ADC conversion data (unit: mV) */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC4_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ICACHE_Init(void);
/* USER CODE BEGIN PFP */
void    configure_MAX77874_buck_conv();   // configure buck converter
//void    configure_DA9070_PMIC();
void 	HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void    send_to_uart(float *pres_map); // send force sensor data to UART and BLE
void 	select_mux_1_pin(uint16_t pin);      // select multiplexer pin
void 	select_mux_2_pin(uint16_t pin);      // select multiplexer pin
void    read_all_sensors();				   // read all the force sensor values
void 	control_valves(int n, int m);      // choose valves to turn on/off
void    find_high_pressure_spots();        // find the two maximum spots of array
void 	valves_autocontrol();
void 	write2SPI(void);
void 	create_pressure_map(float* map);             // create array with all force sensor values

void 	read_uart(void);


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
	uint32_t old_time;
	uint32_t new_time;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the System Power */
  SystemPower_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_I2C1_Init();
  MX_ADC4_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_GPDMA1_Init();
  MX_TIM4_Init();
  MX_ICACHE_Init();
  /* USER CODE BEGIN 2 */
  /*Init serial rx buffer with DMA*/
  HAL_UART_Receive_DMA(&huart4, (uint8_t *)uart_rx_buf, (UART_RX_IDX_MAX + 1));

  /* Perform ADC calibration */
  if (HAL_ADCEx_Calibration_Start(&hadc4, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
   {
     /* Calibration Error */
     Error_Handler();
   }

  /*---------------- Configure MAX77874 Buck Converter --------------------------------------*/

  configure_MAX77874_buck_conv();

  /*---------------- Initialize Bluetooth BM78 module -------------------------------------- */
  HAL_GPIO_WritePin(GPIOC, SW_BTN_Pin, GPIO_PIN_SET); // SW_BTN
  HAL_GPIO_WritePin(GPIOC, WAKE_UP_Pin, GPIO_PIN_SET); // WAKE_UP pin
  HAL_GPIO_WritePin(GPIOC, RST_Pin, GPIO_PIN_SET); // RST pin

  HAL_Delay(100);

  /*---------------- Chip select pin SS of SPI1 and SPI2 is high ----------------------------*/
  HAL_GPIO_WritePin(GPIOA, SPI1_SS_Pin, GPIO_PIN_SET); // SPI1_SS

  /*---------------- Initialize all modules as ON -------------------------------------------*/
/*
  data_written_spi[0] = 0xFF;
  data_written_spi[1] = 0xFF;
  data_written_spi[2] = 0xFF;
  data_written_spi[3] = 0xFF;
  data_written_spi[4] = 0xFF;
*/

  HAL_Delay(1);
  // redistribute
  // Put SS LOW. Activate SPI device.
  HAL_GPIO_WritePin(GPIOA, SPI1_SS_Pin, GPIO_PIN_RESET);

  // Send data
  HAL_SPI_Transmit(&hspi1, data_written_spi, 5, 1000);

  // Bring SS HIGH. Deactivate SPI device.
  HAL_GPIO_WritePin(GPIOA, SPI1_SS_Pin, GPIO_PIN_SET);

  HAL_Delay(1);

  /*---------------- set all the select pins ------------------------------------------------*/
  for (int i=0; i<5; i++)
  {
	  HAL_GPIO_WritePin(GPIOE, select_pins_MUX_1[i], GPIO_PIN_RESET);
  }
  for (int i=0; i<3; i++)
  {
	  HAL_GPIO_WritePin(GPIOD, select_pins_MUX_2[i], GPIO_PIN_RESET);
  }

  for(int i=0; i<NUMBER_OF_SENSORS; i++){
  	  modules_state[i] = 1;
    }

  /*--------------- start the timer ---------------------------------------------------------*/
  /*## Start the Input Capture in interrupt mode ##########################*/
  if(HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  old_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	// loop to create the pressure map

	  read_all_sensors();

	  create_pressure_map(pressure_map);

	  for(int i=0; i<NUMBER_OF_SENSORS; i++){
		  average_pressure_map[i] = average_pressure_map[i] + pressure_map[i];
	  }
	  counter_average++;

	  if (autocontrol) {
		  if(autocontrol_timeout){
			  valves_autocontrol();
			  autocontrol_timeout = 0;
		  }
	  } else {
		  write2SPI();
	  }

	  // use pressure_map and Bluetooth to send data to PC

	  send_to_uart(pressure_map);
	  read_uart();

	  // set all the select pins
	  for (int i=0; i<5; i++)
	  {
		  HAL_GPIO_WritePin(GPIOE, select_pins_MUX_1[i], GPIO_PIN_RESET);
	  }
	  for (int i=0; i<3; i++)
	  {
		  HAL_GPIO_WritePin(GPIOD, select_pins_MUX_2[i], GPIO_PIN_RESET);
	  }

	  //wait
	  do {
		  new_time = HAL_GetTick();
	  }	while ((new_time-old_time) < 10); //100Hz
	  old_time = new_time;
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

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{

  /*
   * Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  HAL_PWREx_DisableUCPDDeadBattery();
}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.ScanConvMode = ADC4_SCAN_DISABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoPowerOff = ADC_LOW_POWER_NONE;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.DMAContinuousRequests = DISABLE;
  hadc4.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  hadc4.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc4.Init.SamplingTimeCommon1 = ADC4_SAMPLETIME_79CYCLES_5;
  hadc4.Init.SamplingTimeCommon2 = ADC4_SAMPLETIME_1CYCLE_5;
  hadc4.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC4_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC4_SAMPLINGTIME_COMMON_1;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

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
  hi2c1.Init.Timing = 0x00303D5B;
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
  hi2c2.Init.Timing = 0x00303D5B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

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

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi1, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 32000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, A4_MUX_Pin|A2_MUX_Pin|A0_MUX_Pin|A3_MUX_Pin
                          |A1_MUX_Pin|DVS_Pin|MODE_Pin|EN_MAX77874_Pin
                          |RIN_N_Pin|WD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_1_Pin|LED_2_Pin|LED_3_Pin|MA2_Pin
                          |MA0_Pin|MA1_Pin|NA0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RST_Pin|WAKE_UP_Pin|SW_BTN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : A4_MUX_Pin A2_MUX_Pin A0_MUX_Pin A3_MUX_Pin
                           A1_MUX_Pin DVS_Pin MODE_Pin EN_MAX77874_Pin
                           RIN_N_Pin WD_Pin */
  GPIO_InitStruct.Pin = A4_MUX_Pin|A2_MUX_Pin|A0_MUX_Pin|A3_MUX_Pin
                          |A1_MUX_Pin|DVS_Pin|MODE_Pin|EN_MAX77874_Pin
                          |RIN_N_Pin|WD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_1_Pin LED_2_Pin LED_3_Pin MA2_Pin
                           MA0_Pin MA1_Pin NA0_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|LED_2_Pin|LED_3_Pin|MA2_Pin
                          |MA0_Pin|MA1_Pin|NA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_Pin WAKE_UP_Pin SW_BTN_Pin */
  GPIO_InitStruct.Pin = RST_Pin|WAKE_UP_Pin|SW_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SYS_FLT_Pin ROUT_N_Pin PWR_FLT_Pin */
  GPIO_InitStruct.Pin = SYS_FLT_Pin|ROUT_N_Pin|PWR_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_SS_Pin */
  GPIO_InitStruct.Pin = SPI1_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_SS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*------------- CUSTOM MADE FUNCTIONS -----------------------------------------------------*/
/*-----------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------*/


void read_all_sensors(){

  int index = -1;
  uint32_t old_timeAD, new_timeAD;
  for (int i=0; i<(NUMBER_OF_SENSORS+1); i++)
  {
	  // choose multiplexer
	  if(i<28){

		  HAL_GPIO_WritePin(GPIOD, NA0_Pin, GPIO_PIN_RESET); // choose multiplexer MUX_1

		  if((i+4) < 12) {
			  select_mux_1_pin(i+4);
			  index++;
		  }
		  if((i+4) == 12) continue;
		  if((i+4) > 12) {
			  select_mux_1_pin(i+4);
			  index++;
		  }

	  }
	  else{

		  HAL_GPIO_WritePin(GPIOD, NA0_Pin, GPIO_PIN_SET);   // choose multiplexer MUX_2

		  select_mux_2_pin(i-28);  // select one mux pin at a time
		  index++;
	  }
	  old_timeAD = HAL_GetTick();
#warning Attention au timing


	  do {
	 	  new_timeAD = HAL_GetTick();
	  }	while ((new_timeAD-old_timeAD) < 10);

	  HAL_ADC_Start(&hadc4); // start the ADC

	  HAL_ADC_PollForConversion(&hadc4, 1000);  // poll for conversion

	  if((i+4) == 12) continue;
	  else{
		  value_adc[index] = HAL_ADC_GetValue(&hadc4);  // store adc value in value
	  }

	  HAL_ADC_Stop(&hadc4);
  }

}

void create_pressure_map(float* map)
{
	int8_t i;
	static float analog_vol[NUMBER_OF_SENSORS];
	// calibration curves, slope & bias



	for(i=0; i<NUMBER_OF_SENSORS; i++){
		analog_vol[i] = value_adc[Mes2Sensor[i+1]-1] * 3.3 / 4095.0;
	}

	// calculate force according to calibration curves
	for(i=0; i<NUMBER_OF_SENSORS; i++){
		map[i] = 0.1*(slope[i+1] * analog_vol[i] + bias[i+1]);
		//map[i] = analog_vol[i];
	}
}

void set_valve (uint8_t numv)
{
	int16_t i;
	int16_t num;
	if (numv == 0) {
		for (i = 0; i < 5; i++) data_written_spi[i] = 0xFF;
	} else {
		num = valve2SPIbit[numv] - 1;
		for (i = 0; i < 5; i++){
			if ((num >= 0) && (num <= 7)){
				data_written_spi[i] |= (1U << num);
			}
			num -= 8;
		}
	}
	/*
	char tmp[4];
	tmp[0] = 'S';
	tmp[1] = numv + '0';
	tmp[2] = '\n';
	HAL_UART_Transmit_IT(&huart4, (uint8_t*) tmp, 3);
	*/
}

void clear_valve (uint8_t numv)
{
	int16_t i;
	int16_t num;
	if (numv == 0) {
		for (i = 0; i < 5; i++) data_written_spi[i] = 0;
	} else {
		num = valve2SPIbit[numv] - 1;
		for (i = 0; i < 5; i++){
			if ((num >= 0) && (num <= 7)){
				data_written_spi[i] &= ~(1U << num );
			}
			num -= 8;
		}
	}
	/*
	char tmp[4];
	tmp[0] = 'C';
	tmp[1] = numv + '0';
	tmp[2] = '\n';
	HAL_UART_Transmit_IT(&huart4, (uint8_t*) tmp, 3);
	*/
}

void send_to_uart(float *pres_map)
{
    //uint8_t buffer[sizeof(float)*NUMBER_OF_SENSORS];
	uint8_t i;
	static float tmp[NUMBER_OF_SENSORS+2];
	tmp[0] = START_BIT;
	for (i=0;i<NUMBER_OF_SENSORS;i++) tmp[i+1] = pres_map[i];
	//for (i=0;i<5;i++) tmp[i+NUMBER_OF_SENSORS+1] = data_written_spi[i];
	tmp[NUMBER_OF_SENSORS+1] = STOP_BIT;
    HAL_UART_Transmit_IT(&huart4, (uint8_t*) tmp, sizeof(float)*(NUMBER_OF_SENSORS+2));
//	HAL_UART_Transmit_IT(&huart4, (uint8_t*) tmp, 10);
}

void read_uart(void)
{
	static uint16_t usart4_dma_ndtr_old = UART_RX_IDX_MAX + 1;
	static uint16_t sbuf_write = 0; //pointeur d'écriture dans le buffer
	static uint16_t sbuf_read = 0; //pointeur de lecture dans le buffer
	static uint8_t uart_rx_state = 0; //machine d'état pour la réception des commandes
	static uint8_t num = 0; //number of the actuator to be set, cleared
	static char action = 0;

	char last_char;
	if (huart4.hdmarx!=NULL) {
		uint16_t usart4_dma_ndtr;
		usart4_dma_ndtr = huart4.hdmarx->Instance->CBR1;
		sbuf_write += ((usart4_dma_ndtr_old - usart4_dma_ndtr) & UART_RX_IDX_MAX);
		usart4_dma_ndtr_old = usart4_dma_ndtr;
	}
	while ((uint16_t)(sbuf_write - sbuf_read) > 0){
		last_char = uart_rx_buf[sbuf_read & UART_RX_IDX_MAX];
		sbuf_read++;
		switch (uart_rx_state) {
			case 0:
				//synchronisation
				if ((last_char == 'S')||(last_char == 'C')) {
					uart_rx_state++;
					num = 0;
					action = last_char;
				}
				break;
			case 1:
				//lecture de la data
				if ((last_char == '\n')||(last_char == '\r')) {
					switch (action) {
						case 'S':
							set_valve(num);
							break;
						case 'C':
							clear_valve(num);
							break;
					}
					uart_rx_state = 0;
				} else {
					//si on a un mauvais caractère, il faut revenir à l'état de sync
					if ((last_char < '0') || (last_char > '9')) {
						uart_rx_state = 0;
					} else {
						num = num * 10 + last_char - '0';
					}
				}
				break;
		}
	}
}


void select_mux_1_pin(uint16_t pin)
{

	  for (int j=0; j<5; j++)
	  {
		if (pin & (1<<j))
		  HAL_GPIO_WritePin(GPIOE, select_pins_MUX_1[j], GPIO_PIN_SET);
		else
		  HAL_GPIO_WritePin(GPIOE, select_pins_MUX_1[j], GPIO_PIN_RESET);
	  }

}

void select_mux_2_pin(uint16_t pin)
{

	  for (int j=0; j<3; j++)
	  {
		if (pin & (1<<j))
		  HAL_GPIO_WritePin(GPIOD, select_pins_MUX_2[j], GPIO_PIN_SET);
		else
		  HAL_GPIO_WritePin(GPIOD, select_pins_MUX_2[j], GPIO_PIN_RESET);
	  }

}


/*---------------- Configure MAX77874 Buck Converter --------------------------------------*/

void    configure_MAX77874_buck_conv()
{

	  uint8_t i2c_data[2];

	  HAL_GPIO_WritePin(GPIOE, DVS_Pin, GPIO_PIN_RESET); // Set Dynamic Voltage Selection DVS to LOW

	  i2c_data[0] = 0x11;          // BUCK0CNFG1 address
	  i2c_data[1] = 0b11000010;    // 8bit data, 40mV/usec ramp rate

	  HAL_I2C_Master_Transmit(&hi2c1, MAX77874_BUCK_CONV_WRITE_ADDRESS, i2c_data, 2, 100); // Enable warm startup ramp rate
	  HAL_Delay(500);

	  i2c_data[0] = 0x21;          // I2C_SDO_VOUT address
	  i2c_data[1] = 70;          // Vout = 0.25V + 5mV*i2c_data[1]

	  HAL_I2C_Master_Transmit(&hi2c1, MAX77874_BUCK_CONV_WRITE_ADDRESS, i2c_data, 2, 100); // Select Output Voltage
	  HAL_Delay(500);

	  HAL_GPIO_WritePin(GPIOE, EN_MAX77874_Pin, GPIO_PIN_SET); // Enable converter. Set Pin EN to HIGH
	  HAL_Delay(1);

}

void write2SPI(void)
{
	// send data through SPI to control all the modules
	// Put SS LOW. Activate SPI device.
	HAL_GPIO_WritePin(GPIOA, SPI1_SS_Pin, GPIO_PIN_RESET);

	// Send data
	HAL_SPI_Transmit(&hspi1, data_written_spi, 5, 1000);

	// Bring SS HIGH. Deactivate SPI device.
	HAL_GPIO_WritePin(GPIOA, SPI1_SS_Pin, GPIO_PIN_SET);
}


void find_high_pressure_spots()
{

	float temp = 0;

	// find two highest pressure spots
	max_1 = average_pressure_map[0];
	max_2 = average_pressure_map[1];

	index_max_1 = 0;
	index_max_2 = 1;

	if(max_1 < max_2){
		temp = max_1;
		max_1 = max_2;
		max_2 = temp;
	}

	for(int i=2; i<NUMBER_OF_SENSORS; i++){
		if(average_pressure_map[i] > max_1){
			max_2 = max_1;
			max_1 = average_pressure_map[i];
			index_max_2 = index_max_1;
			index_max_1 = i;
		}
		else if(average_pressure_map[i]> max_2 && average_pressure_map[i] != max_1){
			max_2 = average_pressure_map[i];
			index_max_2 = i;
		}
	}

}

void control_valves(int m, int n){

	int   flag           = 0;
	int   array_index[4] = {};
	float array[4];
	float a              = 0;
	int   b              = 0;

    // Are there any modules already OFF??
	if(counter_modules_off == 0){ //NO, turn off the two highest PP modules
		modules_state[m] = 0;
		modules_state[n] = 0;

		// set modules state on 64bit UL to zero
		modules_state_uint &= ~(1ULL << m);
		modules_state_uint &= ~(1ULL << n);
	}
	else if(counter_modules_off == 2){ // YES, find their indexes
		for(int j; j<NUMBER_OF_SENSORS; j++){
			if(modules_state[j] == 0){
				if(flag == 0){
					flag = 1;
					index_max_1_previous = j;

				}
				else if(flag == 1){
					flag = 2;
					index_max_2_previous = j;
				}
			}
		}
	}

	// array includes the two previous and two current high PP spots
	array[0] = previous_pressure_map[index_max_1_previous];
	array[1] = previous_pressure_map[index_max_2_previous];
	array[2] = average_pressure_map[m];
	array[3] = average_pressure_map[n];

	array_index[0] = index_max_1_previous;
	array_index[1] = index_max_2_previous;
	array_index[2] = m;
	array_index[3] = n;

	// descending sorting of array and changing place of array_index elements accordingly
	for(int i=0; i<4; i++){
		for(int j=i+1; j<4; j++){
			if(array[i] < array[j]){
				a = array[i];
				array[i] = array[j];
				array[j] = a;

				b = array_index[i];
				array_index[i] = array_index[j];
				array_index[j] = b;
			}
		}
	}

	// set modules state on 64bit UL to one, close two lowest pressure modules
	modules_state_uint = 0xFFFFFFFFFF;

	// set modules state on 64bit UL to zero, open two high pressure modules
	modules_state_uint &= ~(1ULL << valve2SPIbit[array_index[0]]);
	modules_state_uint &= ~(1ULL << valve2SPIbit[array_index[1]]);

   // split the module_state_uint in 5 8bit uint
    data_written_spi[0] = (modules_state_uint & 0x00000000ffULL)      ;
    data_written_spi[1] = (modules_state_uint & 0x000000ff00ULL) >>  8;
    data_written_spi[2] = (modules_state_uint & 0x0000ff0000ULL) >> 16;
    data_written_spi[3] = (modules_state_uint & 0x00ff000000ULL) >> 24;
    data_written_spi[4] = (modules_state_uint & 0xff00000000ULL) >> 32;

	/*-------- CHOOSE MODULES STATE WITH THESE BYTES ------------------*/

    write2SPI();

    counter_modules_off = 2;
}

void valves_autocontrol()
{
	for(int i=0; i<NUMBER_OF_SENSORS; i++){
		average_pressure_map[i] = average_pressure_map[i]/counter_average;
	}

	find_high_pressure_spots();

	control_valves(index_max_1, index_max_2);

	for(int i=0; i<NUMBER_OF_SENSORS; i++){
		previous_pressure_map[i] = average_pressure_map[i];
	}
	counter_average = 0;

	for(int i=0; i<NUMBER_OF_SENSORS; i++){
		average_pressure_map[i] = 0.0;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	autocontrol_timeout = 1;
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
