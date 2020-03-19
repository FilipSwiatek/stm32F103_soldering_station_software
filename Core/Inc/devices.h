/*
 * devices.h
 *
 *  Created on: Mar 17, 2020
 *      Author: Snickers
 */

#ifndef INC_INTERFACES_H_
#define INC_DEVICES_H_

#include <stdbool.h>
#include <stdint.h>

#define SWITCH_1_PIN GPIO_PIN_0
#define SWITCH_2_PIN GPIO_PIN_1
#define SWITCH_3_PIN GPIO_PIN_2
#define SWITCH_4_PIN GPIO_PIN_10
#define POT_SWITCH_PIN GPIO_PIN_11
#define STATUS_LED_PIN GPIO_PIN_13

#define SWITCH_1_PORT GPIOB
#define SWITCH_2_PORT GPIOB
#define SWITCH_3_PORT GPIOB
#define SWITCH_4_PORT GPIOB
#define ENC_SWITCH_PORT GPIOB
#define STATUS_LED_PORT GPIOC

#define ENCODER_FAILURE_TIMEOUT 1000

int8_t EncoderGetOffset(void);
void BuzzerBeep(void);
void HeaterSetDutyCycle(uint8_t percent);
uint16_t GetCurrentTemperature();

// basic interfaces

bool GPIO_Init(void);
bool ADC1_Init(void);
bool I2C1_Init(void);
bool TIM1_Init(void);
bool TIM2_Init(void);
bool USART1_UART_Init(void);
bool TIM3_Init(void);
void StatusLED_Proc(void);

#endif /* INC_DEVICES_H_ */
