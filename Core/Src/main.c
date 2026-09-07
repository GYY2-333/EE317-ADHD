/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "sdadc.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_hid_handler.h"
#include "usbd_customhid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIGNAL_SAMPLE_AUTORELOAD       (24000U - 1U)
#define IMPEDANCE_SAMPLE_AUTORELOAD    (72000U - 1U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static WorkMode applied_mode = MODE_SIGNAL;
static uint8_t impedance_pwm_running = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void APP_ApplyWorkMode(WorkMode mode);
static void APP_UpdateAcquisitionHardware(void);
static void APP_StartImpedanceExcitation(void);
static void APP_StopImpedanceExcitation(void);
static void APP_SynchronizeStreamStart(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void APP_StartImpedanceExcitation(void)
{
  if(impedance_pwm_running == 0U)
  {
    __HAL_TIM_SET_COUNTER(&htim15, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_UPDATE);

    if(HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1) != HAL_OK)
    {
      Error_Handler();
    }
    if(HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1) != HAL_OK)
    {
      (void)HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
      Error_Handler();
    }
    impedance_pwm_running = 1U;
  }
}

static void APP_StopImpedanceExcitation(void)
{
  if(impedance_pwm_running != 0U)
  {
    if(HAL_TIMEx_PWMN_Stop(&htim15, TIM_CHANNEL_1) != HAL_OK)
    {
      Error_Handler();
    }
    if(HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1) != HAL_OK)
    {
      Error_Handler();
    }
    impedance_pwm_running = 0U;
  }
}

static void APP_ApplyWorkMode(WorkMode mode)
{
  uint32_t sample_autoreload;

  if(HAL_TIM_Base_Stop_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if(mode == MODE_IMPEDANCE)
  {
    sample_autoreload = IMPEDANCE_SAMPLE_AUTORELOAD;
    HAL_GPIO_WritePin(Boost_Ctl_GPIO_Port, Boost_Ctl_Pin, GPIO_PIN_RESET);
  }
  else
  {
    sample_autoreload = SIGNAL_SAMPLE_AUTORELOAD;
    HAL_GPIO_WritePin(Boost_Ctl_GPIO_Port, Boost_Ctl_Pin, GPIO_PIN_SET);
  }

  __HAL_TIM_SET_AUTORELOAD(&htim2, sample_autoreload);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

  if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  applied_mode = mode;
}

static void APP_SynchronizeStreamStart(void)
{
  uint32_t primask;

  if(HAL_TIM_Base_Stop_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  APP_StopImpedanceExcitation();

  __HAL_TIM_SET_COUNTER(&htim15, 0U);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_UPDATE);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

  primask = __get_PRIMASK();
  __disable_irq();

  if(current_mode == MODE_IMPEDANCE)
  {
    APP_StartImpedanceExcitation();
  }
  if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  /* 最后进入流状态，保证 TIM2 不会在激励启动前写入首个样本。 */
  comm_state = STATE_STREAMING;
  __set_PRIMASK(primask);
}

static void APP_UpdateAcquisitionHardware(void)
{
  uint8_t excitation_required;

  if(current_mode != applied_mode)
  {
    APP_StopImpedanceExcitation();
    APP_ApplyWorkMode(current_mode);
  }

  if(comm_state == STATE_START_STREAMING)
  {
    APP_SynchronizeStreamStart();
    return;
  }

  excitation_required = ((current_mode == MODE_IMPEDANCE) &&
                         (comm_state == STATE_STREAMING)) ? 1U : 0U;

  if((excitation_required != 0U) && (impedance_pwm_running == 0U))
  {
    APP_StartImpedanceExcitation();
  }
  else if(excitation_required == 0U)
  {
    APP_StopImpedanceExcitation();
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //这是要提交git的代码
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_SDADC3_Init();
  MX_TIM15_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* USB枚举后保持模拟电源关闭；收到上位机有效命令后再由协议层吸合继电器。 */
  HAL_GPIO_WritePin(Boost_Ctl_GPIO_Port, Boost_Ctl_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Relay_Ctl_GPIO_Port, Relay_Ctl_Pin, GPIO_PIN_RESET);

  /* CONF0为单端零参考，CONF1为差分输入；启动时校准两套配置。 */
  if(HAL_SDADC_CalibrationStart(&hsdadc3, SDADC_CALIBRATION_SEQ_2) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_SDADC_PollForCalibEvent(&hsdadc3, HAL_MAX_DELAY) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_SDADC_Start(&hsdadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /* TIM15 只在阻抗流模式开启；TIM2 默认按信号模式采样。 */
  if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    while(APP_DeviceCustomHIDProcessNextReport() != 0U)
    {
      /* 一次处理完队列中的报告，保证 0xF8 先于紧随其后的 0x80 生效。 */
    }

    APP_UpdateAcquisitionHardware();

    if((comm_state == STATE_STREAMING) &&
    (USB_BuildAndSendDataFrame() != 0U))
    {
			static uint8_t LED_Count = 0;
			if(LED_Count==0)HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			LED_Count--;
    }
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_SDADC;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.SdadcClockSelection = RCC_SDADCSYSCLK_DIV12;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG3);
}

/* USER CODE BEGIN 4 */
/* TIM2: 信号模式 3kHz，阻抗模式 1kHz；每 30 点形成一帧。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
    if(comm_state == STATE_STREAMING)
    {
      int16_t adc_value;

      adc_value = (int16_t)HAL_SDADC_GetValue(&hsdadc3);
      APP_PushADCSample((uint16_t)(((int32_t)adc_value+32768)*0.95));
    }
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
#ifdef USE_FULL_ASSERT
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
