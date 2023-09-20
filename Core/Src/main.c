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
#include <string.h>
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

#define START_BIT						 15615.0
#define STOP_BIT						 15700.0
/* Value of analog reference voltage (Vref+), connected to analog voltage   */
/* supply Vdda (unit: mV).                                                  */
#define VDDA_APPLI                       (3300UL)

/* Init variable out of expected ADC conversion data range */
#define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(ADC1, LL_ADC_RESOLUTION_12B) + 1)
/* RX buffer size -1*/
#define UART_RX_IDX_MAX 				 127
#define BM78_I2C_ADDRESS 				 0x28 //BM78 Bluetooth module I2C address ??? not sure
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t 		 value_adc[NUMBER_OF_SENSORS];    // force sensor values read by ADC in every measurement
uint32_t		 value_BAT;                       // state of battery voltage
uint64_t         modules_state_uint = 0xFFFFFFFF;
char			 uart_rx_buf[UART_RX_IDX_MAX+1];  //uart rx buffer

static volatile  uint8_t  autocontrol_timeout = 0;
static uint8_t   autocontrol            = 0; 				//if true, triggers auto control, if not does manual control

static float 	 pressure_map[NUMBER_OF_SENSORS];
static float     average_pressure_map[NUMBER_OF_SENSORS];
static float     previous_pressure_map[NUMBER_OF_SENSORS];
static int8_t    modules_state[NUMBER_OF_SENSORS];
static float 	 max_1 				    = 0.0;
static float 	 max_2 				    = 0.0;
static int 		 counter_average        = 0;  // nb of TIMER1 cycles
static int		 index_max_1			= 0;
static int 		 index_max_2			= 0;
static int       index_max_1_previous   = 0;
static int       index_max_2_previous   = 0;
static int 		 counter_modules_off    = 0;
const  int 		 select_pins_MUX[5]     = {A0_MUX_Pin, A1_MUX_Pin, A2_MUX_Pin, A3_MUX_Pin, A4_MUX_Pin};
const  uint16_t  value_dac              = 8; // DAC output value
const  char*     BM78_SetNameCmd        = "RIGHT_Insole\r\n"; // Replace with RIGHT/LEFT for each insole
static int 		 valve_pins[31]         = {V01_Pin, V02_Pin, V03_Pin, V04_Pin, V05_Pin, V06_Pin, V07_Pin, V08_Pin, V09_Pin, V10_Pin, V11_Pin,
										   V12_Pin, V13_Pin, V14_Pin, V15_Pin, V16_Pin, V17_Pin, V18_Pin, V19_Pin, V20_Pin, V21_Pin,
										   V22_Pin, V23_Pin, V24_Pin, V25_Pin, V26_Pin, V27_Pin, V28_Pin, V29_Pin, V30_Pin, V31_Pin};
GPIO_TypeDef*    pin_port[31]           = {GPIOE, GPIOE, GPIOB, GPIOE, GPIOC, GPIOB, GPIOD, GPIOD, GPIOD, GPIOD, GPIOB,
										   GPIOD, GPIOA, GPIOA, GPIOA, GPIOC, GPIOC, GPIOC, GPIOD, GPIOC, GPIOC,
										   GPIOA, GPIOD, GPIOD, GPIOD, GPIOD, GPIOC, GPIOC, GPIOD, GPIOD, GPIOD};

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

// ------------- CALIBRATION SLOPE CHARACTERISTICS ------------------------------- //
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
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void    configure_MAX77874_buck_conv();    // configure buck converter
void    send_to_uart(float *pres_map);     // send force sensor data to UART and BLE
void 	select_mux_pin(uint16_t pin);      // select multiplexer pin
void    read_all_sensors();				   // read all the force sensor values
void 	control_valves(int n, int m);      // choose valves to turn on/off
void    find_high_pressure_spots();        // find the two maximum spots of array
void 	valves_autocontrol();
void 	create_pressure_map(float* map);   // create array with all force sensor values
void 	read_uart(void);
void    configure_bluetooth();             // configure bluetooth
void    select_ADC_channel();
HAL_StatusTypeDef BM78_SetDeviceName();
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
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /*Init serial rx buffer with DMA*/
  HAL_UART_Receive_DMA(&huart1, (uint8_t *)uart_rx_buf, (UART_RX_IDX_MAX + 1));

  /*---------------- Perform ADC calibration ----------------------------------------------- */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
   {
     /* Calibration Error */
     Error_Handler();
   }

  /*---------------- Configure MAX77874 Buck Converters ------------------------------------ */

  configure_MAX77874_buck_conv();

  /*---------------- Change Bluetooth module name ------------------------------------------ */

   if (BM78_SetDeviceName() != HAL_OK) {
     Error_Handler();
   }

   HAL_Delay(1000); // Delay for 1 second between commands (adjust as needed)

  /*---------------- Initialize Bluetooth BM78 module -------------------------------------- */

  configure_bluetooth();
  HAL_Delay(100);

  /*---------------- set all the select pins ----------------------------------------------- */
  for (int i=0; i<5; i++)
  {
	  if (i<2)	  HAL_GPIO_WritePin(GPIOB, select_pins_MUX[i], GPIO_PIN_RESET);
	  else   	  HAL_GPIO_WritePin(GPIOE, select_pins_MUX[i], GPIO_PIN_RESET);
  }


  for(int i=0; i<NUMBER_OF_SENSORS; i++){
  	  modules_state[i] = 1;  // set all modules state in the state storing array to 1
  	  HAL_GPIO_WritePin(pin_port[i], valve_pins[i], GPIO_PIN_SET);
   }

  /*--------------- start the timer -------------------------------------------------------- */
  /*## Start the Input Capture in interrupt mode ##########################*/
  if(HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  old_time = HAL_GetTick();

  /*--------------- start DAC -------------------------------------------------------------- */
  if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, value_dac) != HAL_OK)
  {
    /* Setting value Error */
    Error_Handler();
  }

  /* -------------- start opamps ------------------------------------------------------------*/
  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
	  }

	  // use pressure_map and Bluetooth to send data to PC

	  send_to_uart(pressure_map);
	  read_uart();

	  // set all the select pins
	  for (int i=0; i<5; i++)
	  {
		  HAL_GPIO_WritePin(GPIOE, select_pins_MUX[i], GPIO_PIN_RESET);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_14B;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_5CYCLE;
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
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};
  DAC_AutonomousModeConfTypeDef sAutonomousMode = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Autonomous Mode
  */
  sAutonomousMode.AutonomousModeState = DAC_AUTONOMOUS_MODE_DISABLE;
  if (HAL_DACEx_SetConfigAutonomousMode(&hdac1, &sAutonomousMode) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hi2c1.Init.Timing = 0x00000E14;
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
  hi2c2.Init.Timing = 0x00000E14;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00000E14;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerSupplyRange = OPAMP_POWERSUPPLY_HIGH;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALPOWER_NORMALSPEED;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_4;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerSupplyRange = OPAMP_POWERSUPPLY_HIGH;
  hopamp2.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMALPOWER_NORMALSPEED;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 32000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, V02_Pin|EN_Pin|SYS_CONF_0_Pin|A2_MUX_Pin
                          |A3_MUX_Pin|A4_MUX_Pin|V04_Pin|V01_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EAN_Pin|V05_Pin|V27_Pin|V28_Pin
                          |V20_Pin|V21_Pin|V16_Pin|V17_Pin
                          |V18_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DVS_Pin|V22_Pin|V13_Pin|V14_Pin
                          |V15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, A0_MUX_Pin|A1_MUX_Pin|SYS_CONF_4_Pin|RST_Pin
                          |WAKE_UP_Pin|V11_Pin|V03_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SW_BTN_Pin|V30_Pin|V31_Pin|V24_Pin
                          |V23_Pin|V29_Pin|V26_Pin|V25_Pin
                          |V19_Pin|V08_Pin|V07_Pin|V10_Pin
                          |V09_Pin|V12_Pin|V06_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : V02_Pin EN_Pin SYS_CONF_0_Pin A2_MUX_Pin
                           A3_MUX_Pin A4_MUX_Pin V04_Pin V01_Pin */
  GPIO_InitStruct.Pin = V02_Pin|EN_Pin|SYS_CONF_0_Pin|A2_MUX_Pin
                          |A3_MUX_Pin|A4_MUX_Pin|V04_Pin|V01_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : EAN_Pin V05_Pin V27_Pin V28_Pin
                           V20_Pin V21_Pin V16_Pin V17_Pin
                           V18_Pin */
  GPIO_InitStruct.Pin = EAN_Pin|V05_Pin|V27_Pin|V28_Pin
                          |V20_Pin|V21_Pin|V16_Pin|V17_Pin
                          |V18_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DVS_Pin V22_Pin V13_Pin V14_Pin
                           V15_Pin */
  GPIO_InitStruct.Pin = DVS_Pin|V22_Pin|V13_Pin|V14_Pin
                          |V15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : A0_MUX_Pin A1_MUX_Pin SYS_CONF_4_Pin RST_Pin
                           WAKE_UP_Pin V11_Pin V03_Pin */
  GPIO_InitStruct.Pin = A0_MUX_Pin|A1_MUX_Pin|SYS_CONF_4_Pin|RST_Pin
                          |WAKE_UP_Pin|V11_Pin|V03_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STIND0_Pin STIND1_Pin */
  GPIO_InitStruct.Pin = STIND0_Pin|STIND1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE12 PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_BTN_Pin V30_Pin V31_Pin V24_Pin
                           V23_Pin V29_Pin V26_Pin V25_Pin
                           V19_Pin V08_Pin V07_Pin V10_Pin
                           V09_Pin V12_Pin V06_Pin */
  GPIO_InitStruct.Pin = SW_BTN_Pin|V30_Pin|V31_Pin|V24_Pin
                          |V23_Pin|V29_Pin|V26_Pin|V25_Pin
                          |V19_Pin|V08_Pin|V07_Pin|V10_Pin
                          |V09_Pin|V12_Pin|V06_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*------------- CUSTOM MADE FUNCTIONS -----------------------------------------------------*/
/*-----------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------*/


void read_all_sensors(){

  uint32_t old_timeAD, new_timeAD;
  for (int i=0; i<(NUMBER_OF_SENSORS+1); i++)
  {

	  // configure mux pins

	  select_mux_pin(i);


	  old_timeAD = HAL_GetTick();

	  //warning Attention au timing


	  do {
	 	  new_timeAD = HAL_GetTick();
	  }	while ((new_timeAD-old_timeAD) < 10);

	  HAL_ADC_Start(&hadc1); // start the ADC

	  HAL_ADC_PollForConversion(&hadc1, 1000);  // poll for conversion

	  value_adc[i] = HAL_ADC_GetValue(&hadc1);  // store adc value in value

	  HAL_ADC_Stop(&hadc1);
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

/*void set_valve (uint8_t numv)
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

	char tmp[4];
	tmp[0] = 'S';
	tmp[1] = numv + '0';
	tmp[2] = '\n';
	HAL_UART_Transmit_IT(&huart4, (uint8_t*) tmp, 3);
	*//*
}*/

/*void clear_valve (uint8_t numv)
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

	char tmp[4];
	tmp[0] = 'C';
	tmp[1] = numv + '0';
	tmp[2] = '\n';
	HAL_UART_Transmit_IT(&huart4, (uint8_t*) tmp, 3);
	*/
//}

void send_to_uart(float *pres_map)
{
    //uint8_t buffer[sizeof(float)*NUMBER_OF_SENSORS];
	uint8_t i;
	static float tmp[NUMBER_OF_SENSORS+2];
	tmp[0] = START_BIT;
	for (i=0;i<NUMBER_OF_SENSORS;i++) tmp[i+1] = pres_map[i];
	//for (i=0;i<5;i++) tmp[i+NUMBER_OF_SENSORS+1] = data_written_spi[i];
	tmp[NUMBER_OF_SENSORS+1] = STOP_BIT;
    HAL_UART_Transmit_IT(&huart1, (uint8_t*) tmp, sizeof(float)*(NUMBER_OF_SENSORS+2));
//	HAL_UART_Transmit_IT(&huart4, (uint8_t*) tmp, 10);
}

void read_uart(void)
{
	static uint16_t usart1_dma_ndtr_old = UART_RX_IDX_MAX + 1;
	static uint16_t sbuf_write = 0; //pointeur d'écriture dans le buffer
	static uint16_t sbuf_read = 0; //pointeur de lecture dans le buffer
	static uint8_t uart_rx_state = 0; //machine d'état pour la réception des commandes
	static uint8_t num = 0; //number of the actuator to be set, cleared
	static char action = 0;

	char last_char;
	if (huart1.hdmarx!=NULL) {
		uint16_t usart1_dma_ndtr;
		usart1_dma_ndtr = huart1.hdmarx->Instance->CBR1;
		sbuf_write += ((usart1_dma_ndtr_old - usart1_dma_ndtr) & UART_RX_IDX_MAX);
		usart1_dma_ndtr_old = usart1_dma_ndtr;
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
							//set_valve(num);
							break;
						case 'C':
							//clear_valve(num);
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


void select_mux_pin(uint16_t pin)
{

	  for (int j=0; j<5; j++)
	  {
		if (pin & (1<<j))
		  HAL_GPIO_WritePin(GPIOE, select_pins_MUX[j], GPIO_PIN_SET);
		else
		  HAL_GPIO_WritePin(GPIOE, select_pins_MUX[j], GPIO_PIN_RESET);
	  }

}




/*---------------- Configure MAX77874 Buck Converter --------------------------------------*/

void    configure_MAX77874_buck_conv()
{

	  uint8_t i2c_data[2];

	  HAL_GPIO_WritePin(GPIOA, DVS_Pin, GPIO_PIN_RESET); // Set Dynamic Voltage Selection DVS to LOW

	  i2c_data[0] = 0x11;          // BUCK0CNFG1 address
	  i2c_data[1] = 0b11000010;    // 8bit data, 40mV/usec ramp rate

	  HAL_I2C_Master_Transmit(&hi2c1, MAX77874_BUCK_CONV_WRITE_ADDRESS, i2c_data, 2, 100); // Enable warm startup ramp rate
	  HAL_I2C_Master_Transmit(&hi2c2, MAX77874_BUCK_CONV_WRITE_ADDRESS, i2c_data, 2, 100); // Enable warm startup ramp rate
	  HAL_I2C_Master_Transmit(&hi2c3, MAX77874_BUCK_CONV_WRITE_ADDRESS, i2c_data, 2, 100); // Enable warm startup ramp rate

	  HAL_Delay(500);

	  i2c_data[0] = 0x21;          // I2C_SDO_VOUT address
	  i2c_data[1] = 70;            // Vout = 0.25V + 5mV*i2c_data[1]

	  HAL_I2C_Master_Transmit(&hi2c1, MAX77874_BUCK_CONV_WRITE_ADDRESS, i2c_data, 2, 100); // Select Output Voltage
	  HAL_I2C_Master_Transmit(&hi2c2, MAX77874_BUCK_CONV_WRITE_ADDRESS, i2c_data, 2, 100); // Select Output Voltage
	  HAL_I2C_Master_Transmit(&hi2c3, MAX77874_BUCK_CONV_WRITE_ADDRESS, i2c_data, 2, 100); // Select Output Voltage

	  HAL_Delay(500);

	  HAL_GPIO_WritePin(GPIOE, EN_Pin, GPIO_PIN_SET); // Enable converter. Set Pin EN to HIGH
	  HAL_Delay(1);

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

	  	HAL_GPIO_WritePin(pin_port[m], valve_pins[m], GPIO_PIN_RESET);
	  	HAL_GPIO_WritePin(pin_port[n], valve_pins[n], GPIO_PIN_RESET);


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

	HAL_GPIO_WritePin(pin_port[array_index[0]], valve_pins[array_index[0]], GPIO_PIN_RESET);
  	HAL_GPIO_WritePin(pin_port[array_index[1]], valve_pins[array_index[1]], GPIO_PIN_RESET);

	modules_state[array_index[0]] = 0;
	modules_state[array_index[1]] = 0;

	// set modules state on 64bit UL to one, close two lowest pressure modules
	modules_state_uint = 0xFFFFFFFFFF;

	// set modules state on 64bit UL to zero, open two high pressure modules
	modules_state_uint &= ~(1ULL << valve2SPIbit[array_index[0]]);
	modules_state_uint &= ~(1ULL << valve2SPIbit[array_index[1]]);

   // split the module_state_uint in 5 8bit uint
    //data_written_spi[0] = (modules_state_uint & 0x00000000ffULL)      ;
    //data_written_spi[1] = (modules_state_uint & 0x000000ff00ULL) >>  8;
    //data_written_spi[2] = (modules_state_uint & 0x0000ff0000ULL) >> 16;
    //data_written_spi[3] = (modules_state_uint & 0x00ff000000ULL) >> 24;
    //data_written_spi[4] = (modules_state_uint & 0xff00000000ULL) >> 32;

	/*-------- CHOOSE MODULES STATE WITH THESE BYTES ------------------*/


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

void    configure_bluetooth(){

	  HAL_GPIO_WritePin(GPIOD, SW_BTN_Pin, GPIO_PIN_SET); // SW_BTN
	  HAL_GPIO_WritePin(GPIOB, WAKE_UP_Pin, GPIO_PIN_SET); // WAKE_UP pin
	  HAL_GPIO_WritePin(GPIOB, RST_Pin, GPIO_PIN_SET); // RST pin

}

void    select_ADC_channel(uint32_t channel){
	  ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = channel;
	  sConfig.Rank = 1;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

HAL_StatusTypeDef BM78_SetDeviceName() {
    // Define buffer for I2C communication
    uint8_t i2cData[32]; // Adjust the buffer size as needed

    // Convert the command string to bytes
    strcpy((char*)i2cData, BM78_SetNameCmd);

    // Perform I2C write operation to send the command to BM78
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, BM78_I2C_ADDRESS, i2cData, strlen(BM78_SetNameCmd), HAL_MAX_DELAY);

    return status;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance == TIM1)	autocontrol_timeout = 1;
	else if(htim->Instance == TIM2){
		select_ADC_channel(ADC_CHANNEL_3);

		HAL_ADC_Start(&hadc1); // start the ADC
		HAL_ADC_PollForConversion(&hadc1, 1000);  // poll for conversion
		value_BAT = HAL_ADC_GetValue(&hadc1);  // store adc value in value
		HAL_ADC_Stop(&hadc1);

		select_ADC_channel(ADC_CHANNEL_2);
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
