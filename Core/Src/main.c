/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <stdbool.h>
#include "ssd1306.h"
#include "devices.h"
#include <stdio.h>

#define STARTUP_TEMP 270
#define SETUP_MODE_TIMEOUT 2000
#define VERTICAL_DISPLAY_OFFSET 17
#define HORIZONTAL_DISPLAY_OFFSET 25
#define SETUP_BLINKING_DIGIT_TIME 200

volatile static bool wasEncoderButtonPressed;
static uint16_t TemporarlySetTemp = STARTUP_TEMP; // temperature used in setup mode
static uint16_t SetTemp = STARTUP_TEMP; // temperature set for controller. This is the important one for PID
static bool isNowSetupMode; // determines if TemporarilySetTemp should be written to SetTemp variable
static uint8_t setDigit; // determines which digit is set by encoder (tens or ones)

bool static SystemClock_Config(void);
void static TempSetupProc(void);
void static DisplayProc(void);

int main(void){
  HAL_Init();
  if(!SystemClock_Config()) Error_Handler();
  if(!GPIO_Init()) Error_Handler();
  if(!ADC1_Init()) Error_Handler();
  if(!I2C1_Init()) Error_Handler();
  ssd1306_Init();
  if(!TIM1_Init()) Error_Handler();
  if(!TIM2_Init()) Error_Handler();
  if(!TIM3_Init()) Error_Handler();
  if(!USART1_UART_Init()) Error_Handler();
  while (1){
	  StatusLED_Proc();
	  TempSetupProc();
	  DisplayProc();
  }
}

bool static SystemClock_Config(void){
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    return false;
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){
	  return false;
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK){
	  return false;
  }
  return true;
}

void Error_Handler(void){
	HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_RESET); // turn status diode on
	while(1);
}


void static TempSetupProc(void){
	static uint32_t StartOfSetupModeTimestamp;
	int8_t encoderOffset = 0;

	encoderOffset = EncoderGetOffset();
	//checking if setup mode should start
	if(encoderOffset || wasEncoderButtonPressed){ // if encoder position changed or button has been clicked
		StartOfSetupModeTimestamp = HAL_GetTick();
		isNowSetupMode = true;
	}

	if(isNowSetupMode){
		if(wasEncoderButtonPressed){
				if(setDigit == 0){
					setDigit = 1; // changing digit
				}else{
					setDigit = 0; // changing digit
				}
				wasEncoderButtonPressed = false;
			}

			if(setDigit == 1){
				TemporarlySetTemp += 10 * encoderOffset;
			}else{
				TemporarlySetTemp += encoderOffset;
			}

			if(TemporarlySetTemp < 150){
				TemporarlySetTemp = 150;
			}else if(TemporarlySetTemp > 450){
				TemporarlySetTemp = 450;
			}
	}else{ // if setup mode terminated
		SetTemp = TemporarlySetTemp; // changing Set Temperature
	}

	if(HAL_GetTick() - StartOfSetupModeTimestamp > SETUP_MODE_TIMEOUT){
		setDigit = 0; //returning to less significant digit
		isNowSetupMode = false; // exit setup mode
	}
}

void static DisplayProc(void){
	static uint32_t lastBlinkTimeStamp;
	uint16_t temperature;
	uint16_t cursor_backup[2]={0,0}; // value needed to store current cursor position
	char temperatureString[4];  // temperature i Celsius degrees
	char pseudoDegree = 'o'; // pseudo-degree
	char CelsiusUnit='C'; // Celsius unit
	ssd1306_Fill(Black); // black background

	if(isNowSetupMode){
		temperature = TemporarlySetTemp;
	}else{
		temperature = GetCurrentTemperature();
	}
	sprintf(temperatureString, "%d", temperature);

	if(isNowSetupMode && HAL_GetTick() - lastBlinkTimeStamp > SETUP_BLINKING_DIGIT_TIME ){
		switch(setDigit){
		case 0:
			temperatureString[2] = ' ';
			break;
		case 1:
			temperatureString[1] = ' ';
			break;
		default:
			break;
		}
	}
	ssd1306_SetCursor(HORIZONTAL_DISPLAY_OFFSET, 3 + VERTICAL_DISPLAY_OFFSET); // setting offset for cursor to reach pseudo- degree effect using 'o'
	ssd1306_WriteString(temperatureString, Font_16x26, White); //wrinting decimal temperature value in Celsius degrees
	ssd1306_GetCursor(cursor_backup); // getting current cursor position
	ssd1306_SetCursor(cursor_backup[0]+2, 0 + VERTICAL_DISPLAY_OFFSET); // switching vertical position for pseudo-degree
	ssd1306_WriteChar(pseudoDegree, Font_11x18, White); // printing pseudo degree character
	ssd1306_GetCursor(cursor_backup); // getting current cursor position
	ssd1306_SetCursor(cursor_backup[0]-2, 3 + VERTICAL_DISPLAY_OFFSET); // switching vertical position back to previous
	ssd1306_WriteChar(CelsiusUnit, Font_16x26, White); // Writing 'C' character

	ssd1306_UpdateScreen();
	if(HAL_GetTick() - lastBlinkTimeStamp > 1.1 * SETUP_BLINKING_DIGIT_TIME){
					lastBlinkTimeStamp = HAL_GetTick();
				}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == POT_SWITCH_PIN ){
		wasEncoderButtonPressed = true;
	}
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
