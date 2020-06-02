/*
 * devices.c
 *
 *  Created on: Mar 17, 2020
 *      Author: Snickers
 */

#include <devices.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"


ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;


// Heater PWM
void SetHeaterPower(uint16_t perMile){
	if(perMile > 1000 ) perMile = 1000;
	TIM2->CCR3 = perMile;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, perMile);
}
void TurnHeaterOn(){
	HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_3);
}
void TurnHeaterOff(){
	HAL_TIMEx_PWMN_Stop(&htim2, TIM_CHANNEL_3);
}

// Buzzer


void TurnBuzzerOn(){
	HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_1);
}

void TurnBuzzerOff(){
	HAL_TIMEx_PWMN_Stop(&htim3, TIM_CHANNEL_1);
}

void BuzzerSetFrequency(uint16_t kiloHerzes){
	__HAL_TIM_SET_PRESCALER(&htim3, 1000000/kiloHerzes);
}


// Encoder
int8_t EncoderGetOffset(void){
	int32_t enc_val = ((int32_t)TIM1->CNT)-0xFF;
	static uint32_t lastNonFailureEncoderStepTimestamp;

	if(enc_val >= 4 || enc_val <= -4){
		TIM1->CNT -= enc_val;
		enc_val = enc_val / 4;
		lastNonFailureEncoderStepTimestamp = HAL_GetTick();

	}else{

		enc_val = 0;
	}

	if(HAL_GetTick() - lastNonFailureEncoderStepTimestamp > ENCODER_FAILURE_TIMEOUT){
		TIM1->CNT = 0xFF;
	}
	return enc_val;
}

uint16_t GetCurrentTemperature(){
	//TODO - to develop
	return 250;
}

//------------------Basic IOs

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

 bool ADC1_Init(void){
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    return false;
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    return false;
  }
  return true;
}

 bool I2C1_Init(void){
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
    return false;
  }
return true;
}

 bool TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_BOTHEDGE;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK){
    return false;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK){
    return false;
  }
  htim1.Instance->CNT = 0xFF;
  htim1.Instance->CR1 |= TIM_CR1_CEN;
return true;
}

 bool TIM2_Init(void){

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 36; // because TIM clock is 36MHz
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000; // because after prescaler clock is 1MHz, and desired PWM frequency is 1kHz
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK){
    return false;
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK){
    return false;
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK){
	  return false;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    return false;
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK){
    return false;
  }
  HAL_TIM_MspPostInit(&htim2);
return true;
}

 bool TIM3_Init(void){

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 36;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    return false;
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
	  return false;
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
	  return false;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
	  return false;
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
	  return false;
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 18);


  return true;

}

 bool USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK){
    return false;
  }
return true;
}

 bool GPIO_Init(void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  //buttons' and status led's  GPIOs clock enable
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  // initialize buttons
  GPIO_InitStruct.Pin = SWITCH_1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SWITCH_1_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SWITCH_2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SWITCH_2_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SWITCH_3_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SWITCH_3_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SWITCH_4_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SWITCH_4_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ENCODER_SWITCH_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENC_SWITCH_PORT, &GPIO_InitStruct);

  //initialize status diode
  GPIO_InitStruct.Pin = STATUS_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_SET); // initialize diode as turned off
  HAL_GPIO_Init(STATUS_LED_PORT, &GPIO_InitStruct);

  /* EXTI interrupt init*/
   HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(EXTI0_IRQn);

   HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(EXTI1_IRQn);

   HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(EXTI2_IRQn);

   HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

return true;
}


 void StatusLED_Proc(void){
 	static uint32_t previousCallTimestamp;
 	if((HAL_GetTick() - previousCallTimestamp) > 1000 ){
 		HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN); // toggle status diode state
 		previousCallTimestamp = HAL_GetTick();
 	}
 }
