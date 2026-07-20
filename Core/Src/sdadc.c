/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdadc.c
  * @brief   This file provides code for the configuration
  *          of the SDADC instances.
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
#include "sdadc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SDADC_HandleTypeDef hsdadc3;

/* SDADC3 init function */
void MX_SDADC3_Init(void)
{

  /* USER CODE BEGIN SDADC3_Init 0 */

  /* USER CODE END SDADC3_Init 0 */

  SDADC_ConfParamTypeDef ConfParamStruct = {0};

  /* USER CODE BEGIN SDADC3_Init 1 */

  /* USER CODE END SDADC3_Init 1 */

  /** Configure the SDADC low power mode, fast conversion mode,
  slow clock mode and SDADC1 reference voltage
  */
  hsdadc3.Instance = SDADC3;
  hsdadc3.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
  hsdadc3.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
  hsdadc3.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
  hsdadc3.Init.ReferenceVoltage = SDADC_VREF_VREFINT2;
  if (HAL_SDADC_Init(&hsdadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure The Regular Mode
  */
  if (HAL_SDADC_SelectRegularTrigger(&hsdadc3, SDADC_SOFTWARE_TRIGGER) != HAL_OK)
  {
    Error_Handler();
  }

  /** Set parameters for SDADC configuration 0 Register
  */
  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_ZERO_REFERENCE;
  ConfParamStruct.Gain = SDADC_GAIN_1_2;
  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VSSA;
  ConfParamStruct.Offset = 0;
  if (HAL_SDADC_PrepareChannelConfig(&hsdadc3, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Set parameters for SDADC configuration 1 Register
  */
  ConfParamStruct.InputMode = SDADC_INPUT_MODE_DIFF;
  if (HAL_SDADC_PrepareChannelConfig(&hsdadc3, SDADC_CONF_INDEX_1, &ConfParamStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Regular Channel
  */
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc3, SDADC_CHANNEL_8, SDADC_CONF_INDEX_0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SDADC_ConfigChannel(&hsdadc3, SDADC_CHANNEL_8, SDADC_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDADC3_Init 2 */

  /* USER CODE END SDADC3_Init 2 */

}

void HAL_SDADC_MspInit(SDADC_HandleTypeDef* sdadcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(sdadcHandle->Instance==SDADC3)
  {
  /* USER CODE BEGIN SDADC3_MspInit 0 */

  /* USER CODE END SDADC3_MspInit 0 */
    /* SDADC3 clock enable */
    __HAL_RCC_SDADC3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SDADC3 GPIO Configuration
    PB14     ------> SDADC3_AIN8P
    PB15     ------> SDADC3_AIN7P
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SDADC3_MspInit 1 */

  /* USER CODE END SDADC3_MspInit 1 */
  }
}

void HAL_SDADC_MspDeInit(SDADC_HandleTypeDef* sdadcHandle)
{

  if(sdadcHandle->Instance==SDADC3)
  {
  /* USER CODE BEGIN SDADC3_MspDeInit 0 */

  /* USER CODE END SDADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SDADC3_CLK_DISABLE();

    /**SDADC3 GPIO Configuration
    PB14     ------> SDADC3_AIN8P
    PB15     ------> SDADC3_AIN7P
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_14|GPIO_PIN_15);

  /* USER CODE BEGIN SDADC3_MspDeInit 1 */

  /* USER CODE END SDADC3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* ---- SDADC 输入模式切换 ---- */
/* conf_index: SDADC_CONF_INDEX_0 = 单端(信号检测), SDADC_CONF_INDEX_1 = 差分(阻抗) */
HAL_StatusTypeDef SDADC_SwitchInputMode(uint32_t conf_index)
{
    HAL_StatusTypeDef status;

    status = HAL_SDADC_Stop(&hsdadc3);
    if(status != HAL_OK)
    {
        return status;
    }

    status = HAL_SDADC_AssociateChannelConfig(&hsdadc3,
                                              SDADC_CHANNEL_8,
                                              conf_index);
    if(status != HAL_OK)
    {
        return status;
    }

    status = HAL_SDADC_ConfigChannel(&hsdadc3,
                                     SDADC_CHANNEL_8,
                                     SDADC_CONTINUOUS_CONV_ON);
    if(status != HAL_OK)
    {
        return status;
    }

    return HAL_SDADC_Start(&hsdadc3);
}

/* USER CODE END 1 */

