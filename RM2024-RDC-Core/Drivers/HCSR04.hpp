#pragma once
#include "AppConfig.h"

#if USE_SR04

#include "main.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

namespace SR04
{
#define TRIG_Pin GPIO_PIN_5
#define TRIG_GPIO_Port GPIOA
#define ECHO_Pin GPIO_PIN_6
#define ECHO_GPIO_Port GPIOA

void delay_us(uint16_t time);

uint16_t HCSR04_Read(void);

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
}

#endif  