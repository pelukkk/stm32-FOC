/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "AS5047P.h"
#include "DRV8302.h"
#include "FOC_utils.h"
#include "bldc_midi.h"
#include "flash.h"
#include "controller_app.h"
#include <stdio.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define BLDC_PWM_FREQ AUDIO_SAMPLE_RATE
// #define BLDC_PWM_FREQ 10000
#define R_SHUNT	0.01f
#define V_OFFSET_A	1.645f
#define V_OFFSET_B	1.657f
#define POLE_PAIR	(7)
#define SPEED_CONTROL_CYCLE	10
#define FOC_TS (1.0f / (float)BLDC_PWM_FREQ)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim10;

osThreadId controlTaskHandle;
osThreadId comTaskHandle;
/* USER CODE BEGIN PV */

foc_t hfoc;
DRV8302_t bldc;
AS5047P_t hencd;

float angle_deg = 0.0f;
float sp_input = 0.0f;
int start_cal = 0;
_Bool com_init_flag = 0;
_Bool calibration_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM10_Init(void);
static void MX_ADC3_Init(void);
void StartControlTask(void const * argument);
void StartComTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/******************************************************************************/

void bldc_config(DRV8302_t *hbldc) {
  DRV8302_GPIO_MPWM_config(hbldc, M_PWM_GPIO_Port, M_PWM_Pin);
  DRV8302_GPIO_MOC_config(hbldc, M_OC_GPIO_Port, M_OC_Pin);
  DRV8302_GPIO_GAIN_config(hbldc, GAIN_GPIO_Port, GAIN_Pin);
  DRV8302_GPIO_DCCAL_config(hbldc, DC_CAL_GPIO_Port, DC_CAL_Pin);
  DRV8302_GPIO_OCTW_config(hbldc, OCTW_GPIO_Port, OCTW_Pin);
  DRV8302_GPIO_FAULT_config(hbldc, FAULT_GPIO_Port, FAULT_Pin);
  DRV8302_GPIO_ENGATE_config(hbldc, EN_GATE_GPIO_Port, EN_GATE_Pin);
  DRV8302_TIMER_config(hbldc, &htim1, BLDC_PWM_FREQ);
  DRV8302_current_sens_config(hbldc, _10VPV, R_SHUNT, V_OFFSET_A, V_OFFSET_B);
  DRV8302_set_mode(hbldc, _6_PWM_MODE, _CYCLE_MODE);

  DRV8302_init(hbldc);

  DRV8302_disable_dc_cal(hbldc);
  DRV8302_enable_gate(hbldc);
}

/******************************************************************************/

float get_power_voltage(void) {
	static float pv_filtered = 0.0f;
  const float filter_alpha = 0.2f; 

	// convert to volt
	float pv = (float)ADC3->JDR1 * ADC_2_POWER_VOLT;

  // Low-pass filter for noise reduction
	pv_filtered = (1.0f - filter_alpha) * pv_filtered + filter_alpha * pv;

	return pv_filtered;
}

/******************************************************************************/

uint32_t get_dt_us(void) {
#if 0
  static uint32_t last_us = 0;

  uint32_t now_us = TIM10->CNT;
  uint32_t elapsed_us = now_us - last_us;
  last_us = now_us;
#else
  uint32_t elapsed_us = TIM10->CNT;
  TIM10->CNT = 0;
#endif
  return elapsed_us;
}

/******************************************************************************/

#define CAL_ITERATION 100

#define VD_CAL 0.6f
#define VQ_CAL 0.0f

void cal_encoder_misalignment(void) {
  open_loop_voltage_control(&hfoc, VD_CAL, VQ_CAL, 0.0f);
  HAL_Delay(500);
  float rad_offset = 0.0f;
  for (int i = 0; i < CAL_ITERATION; i++) {
    rad_offset += DEG_TO_RAD(hencd.angle_filtered);
    HAL_Delay(1);
  }
  open_loop_voltage_control(&hfoc, 0.0f, 0.0f, 0.0f);
  rad_offset = rad_offset / (float)CAL_ITERATION;
  hfoc.m_angle_offset = rad_offset;
  m_config.encd_offset = rad_offset;
}

float error_temp[ERROR_LUT_SIZE] = {0};

void encoder_get_error(void) {
  memset(error_temp, 0, sizeof(error_temp));

  cal_encoder_misalignment();

  for (int i = 0; i < ERROR_LUT_SIZE; i++) {
    float mech_deg = (float)i * (360.0f / (float)ERROR_LUT_SIZE);
    float elec_rad = DEG_TO_RAD(mech_deg * POLE_PAIR);
    open_loop_voltage_control(&hfoc, VD_CAL, VQ_CAL, elec_rad);
    HAL_Delay(5);

    float mech_rad = hfoc.m_angle_rad;
    float raw_delta = elec_rad - hfoc.e_angle_rad;
    float delta = elec_rad - hfoc.e_angle_rad_comp;
    
    raw_delta -= TWO_PI * floorf((raw_delta + PI) / TWO_PI);
    delta -= TWO_PI * floorf((delta + PI) / TWO_PI);
    
    float lut_pos = (mech_rad / TWO_PI) * ERROR_LUT_SIZE;
    int index = (int)(lut_pos);

    while (index < 0) {
      index += ERROR_LUT_SIZE;
    }
    index %= ERROR_LUT_SIZE;

    error_temp[index] = raw_delta;
    
    // debug
    float buffer_val[2];
    // buffer_val[0] = raw_delta;
    // buffer_val[1] = delta;
    buffer_val[0] = delta;
    send_data_float(buffer_val, 1);
  }
  
  for (int i = 0; i < ERROR_LUT_SIZE; i++) {
    if (error_temp[i] == 0) {
      int last_i = i - 1;
      int next_i = i + 1;
      if (last_i < 0) last_i += ERROR_LUT_SIZE;
      if (next_i > ERROR_LUT_SIZE) next_i -= ERROR_LUT_SIZE;
      error_temp[i] = (error_temp[last_i] + error_temp[next_i]) / 2.0f;
    }
  }

  memcpy(m_config.encd_error_comp, error_temp, sizeof(error_temp));

  open_loop_voltage_control(&hfoc, 0.0f, 0.0f, 0.0f);
}

/******************************************************************************/

void svpwm_test(void) {
  open_loop_voltage_control(&hfoc, VD_CAL, VQ_CAL, 0.0f);
  HAL_Delay(500);

  for (int i = 0; i < ERROR_LUT_SIZE; i++) {
    float mech_deg = (float)i * (360.0f / (float)ERROR_LUT_SIZE);
    float elec_rad = DEG_TO_RAD(mech_deg * POLE_PAIR);
    open_loop_voltage_control(&hfoc, VD_CAL, VQ_CAL, elec_rad);
    HAL_Delay(10);
    // float pwm[3];
    // pwm[0] = TIM1->CCR1;
    // pwm[1] = TIM1->CCR2;
    // pwm[2] = TIM1->CCR3;
    // send_data_float(pwm, 3);
    float rad[2];
    rad[0] = hfoc.e_angle_rad_comp;
    rad[1] = elec_rad;
    norm_angle_rad(&rad[0]);
    norm_angle_rad(&rad[1]);
    send_data_float(rad, 2);
  }

  open_loop_voltage_control(&hfoc, 0.0f, 0.0f, 0.0f);
}

/******************************************************************************/

// platformio run --target upload 
// platformio run --target clean
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  printf("FPU aktif!\n");
#else
  printf("FPU tidak aktif!\n");
#endif

  MX_USB_DEVICE_Init();

  read_config_from_flash(&m_config);

  init_trig_lut();
  
  pid_init(&hfoc.id_ctrl, m_config.id_kp, m_config.id_ki, 0.0f, FOC_TS, m_config.id_out_max, m_config.id_e_deadband);
  pid_init(&hfoc.iq_ctrl, m_config.iq_kp, m_config.iq_ki, 0.0f, FOC_TS, m_config.iq_out_max, m_config.iq_e_deadband);
  pid_init(&hfoc.speed_ctrl, m_config.speed_kp, m_config.speed_ki, 0.0f, FOC_TS * SPEED_CONTROL_CYCLE, m_config.speed_out_max, m_config.speed_e_deadband);
  pid_init(&hfoc.pos_ctrl, m_config.pos_kp, m_config.pos_ki, m_config.pos_kd, FOC_TS * SPEED_CONTROL_CYCLE, m_config.pos_out_max, m_config.pos_e_deadband);

  AS5047P_config(&hencd, &hspi1, SPI_CS_GPIO_Port, SPI_CS_Pin);
  AS5047P_start(&hencd);
  HAL_Delay(10);
  AS5047P_start(&hencd);

  bldc_config(&bldc);
	// current sensor
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc2);
	// voltage sensor
	HAL_ADCEx_InjectedStart_IT(&hadc3);

  foc_pwm_init(&hfoc, &(TIM1->CCR3), &(TIM1->CCR2), &(TIM1->CCR1), bldc.pwm_resolution);
  foc_motor_init(&hfoc, POLE_PAIR, 360);

  foc_sensor_init(&hfoc, m_config.encd_offset, REVERSE_DIR); //1.8704f
  foc_gear_reducer_init(&hfoc, 1.0/19.0);
  foc_set_limit_current(&hfoc, 10.0);

  HAL_TIM_Base_Start(&htim10);

  hfoc.control_mode = TEST_MODE;
  // hfoc.control_mode = TORQUE_CONTROL_MODE;
  // hfoc.control_mode = AUDIO_MODE;

  // kp=4.5 kd=0.3

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of controlTask */
  osThreadDef(controlTask, StartControlTask, osPriorityAboveNormal, 0, 256);
  controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);

  /* definition and creation of comTask */
  osThreadDef(comTask, StartComTask, osPriorityLow, 0, 256);
  comTaskHandle = osThreadCreate(osThread(comTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_TRIPLEMODE_INJECSIMULT;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_FALLING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.AutoInjectedConv = ENABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.AutoInjectedConv = ENABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1024;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 5;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 167;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|DC_CAL_Pin|GAIN_Pin|M_OC_Pin
                          |M_PWM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin DC_CAL_Pin GAIN_Pin M_OC_Pin
                           M_PWM_Pin */
  GPIO_InitStruct.Pin = LED_Pin|DC_CAL_Pin|GAIN_Pin|M_OC_Pin
                          |M_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : OCTW_Pin FAULT_Pin */
  GPIO_InitStruct.Pin = OCTW_Pin|FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_GATE_Pin */
  GPIO_InitStruct.Pin = EN_GATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_GATE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

uint32_t dt_us;

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) {
    angle_deg = AS5047P_get_degree(&hencd);
    foc_calc_electric_angle(&hfoc, DEG_TO_RAD(angle_deg));
	}
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
    DRV8302_set_adc_a(&bldc, ADC1->JDR1);
  }
	if (hadc->Instance == ADC2) {
    DRV8302_set_adc_b(&bldc, ADC2->JDR1);
  }
	if (hadc->Instance == ADC3) {
    static uint8_t event_loop_count = 0;

    hfoc.v_bus = get_power_voltage();
    AS5047P_start(&hencd);
    
    switch(hfoc.control_mode) {
      case TORQUE_CONTROL_MODE:
      case SPEED_CONTROL_MODE:
      case POSITION_CONTROL_MODE: {
        DRV8302_get_current(&bldc, &hfoc.ia, &hfoc.ib);
        foc_current_control_update(&hfoc);

        if (event_loop_count > SPEED_CONTROL_CYCLE) {
          event_loop_count = 0;
          dt_us = get_dt_us();
          hfoc.actual_rpm = AS5047P_get_rpm(&hencd, dt_us);
          foc_set_flag();
        }
        event_loop_count++;
        break;
      }
      case AUDIO_MODE: {
        audio_loop(&hfoc);
        break;
      }
      default:
      break;
    }
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartControlTask */
/**
  * @brief  Function implementing the controlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    if (is_foc_ready()) {
      foc_reset_flag();

      switch (hfoc.control_mode) {
      case TORQUE_CONTROL_MODE:
        hfoc.id_ref = 0.0f;
        hfoc.iq_ref = sp_input;
        break;
      case SPEED_CONTROL_MODE: {
        foc_speed_control_update(&hfoc, sp_input);
        break;
      }
      case POSITION_CONTROL_MODE:
        hfoc.actual_angle = AS5047P_get_actual_degree(&hencd);
        foc_position_control_update(&hfoc, sp_input);
        break;
      case CALIBRATION_MODE:
        break;
      case TEST_MODE:
        open_loop_voltage_control(&hfoc, 0.0f, 0.1f, 0.0f);
        break;
      default:
        break;
      }
    }
    
    if (hfoc.control_mode == CALIBRATION_MODE) {
      if (start_cal == 1) {
        encoder_get_error();
        // svpwm_test();
        start_cal = 0;
      }
    }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartComTask */
/**
* @brief Function implementing the comTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartComTask */
void StartComTask(void const * argument)
{
  /* USER CODE BEGIN StartComTask */

  uint32_t debug_tick = HAL_GetTick();
  uint32_t blink_tick = HAL_GetTick();

  motor_mode_t last_mode = CALIBRATION_MODE;

  /* Infinite loop */
  for(;;)
  {
    if (HAL_GetTick() - debug_tick >= 1) {
      debug_tick = HAL_GetTick();
      float data[16];
      uint16_t len = 0;

      switch (hfoc.control_mode) {
      case TORQUE_CONTROL_MODE:
        data[len++] = sp_input;
        data[len++] = hfoc.id_filtered;
        data[len++] = hfoc.iq_filtered;
        break;
      case SPEED_CONTROL_MODE:
        data[len++] = sp_input;
        data[len++] = hfoc.actual_rpm;
        break;
      case POSITION_CONTROL_MODE:
        data[len++] = sp_input;
        data[len++] = hfoc.actual_angle;
        // data[len++] = hfoc.iq;
        break;
      case AUDIO_MODE:
        data[len++] = v_tone;
        break;
      case CALIBRATION_MODE:
        data[len++] = hencd.prev_raw_angle;
        data[len++] = hencd.angle_filtered;
        break;
      }

      send_data_float(data, len);
    }

    if (com_init_flag) {
      last_mode = -1;
      com_init_flag = 0;
    }

    if (calibration_flag) {
      last_mode = -1;
      calibration_flag = 0;
    }

    if (hfoc.control_mode != last_mode) {
      last_mode = hfoc.control_mode;
      float data[16];

      osDelay(2);
      erase_graph(); osDelay(1);
      switch (hfoc.control_mode) {
      case TORQUE_CONTROL_MODE:
        send_data_float(data, 3); osDelay(1);
        change_title("CURRENT CONTROL MODE"); osDelay(1);
        change_legend(0, "Iq_sp"); osDelay(1);
        change_legend(1, "Id"); osDelay(1);
        change_legend(2, "Iq"); osDelay(1);
        break;
      case SPEED_CONTROL_MODE:
        send_data_float(data, 2); osDelay(1);
        change_title("SPEED CONTROL MODE"); osDelay(1);
        change_legend(0, "set point"); osDelay(1);
        change_legend(1, "rpm"); osDelay(1);
        break;
      case POSITION_CONTROL_MODE:
        send_data_float(data, 2); osDelay(1);
        change_title("POSITION CONTROL MODE"); osDelay(1);
        change_legend(0, "set point"); osDelay(1);
        change_legend(1, "actual angle"); osDelay(1);
        // change_legend(2, "iq"); osDelay(1);
        break;
      case CALIBRATION_MODE:
        send_data_float(data, 1); osDelay(1);
        change_title("CALIBRATION MODE"); osDelay(1);
        change_legend(0, "error angle (rad)"); osDelay(1);
        start_cal = 1;
        break;
      case AUDIO_MODE:
        send_data_float(data, 1); osDelay(1);
        change_title("AUDIO MODE"); osDelay(1);
        change_legend(0, "vd"); osDelay(1);
        start_cal = 1;
        break;
      default:
        break;
      }
    }

    if (HAL_GetTick() - blink_tick >= 500) {
      blink_tick = HAL_GetTick();
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
    osDelay(1);
  }
  /* USER CODE END StartComTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
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
  DRV8302_disable_gate(&bldc);
  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(200);
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
