#pragma once
#include "AppConfig.h"

#if USE_HCSR04

#include "main.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

namespace HCSR04
{
    void hcsr04_init();
    
    void delay_us(uint16_t time);

    void SendSingnal();

    uint16_t HCSR04_Read(void);

    void IC_CaptureCallback(TIM_HandleTypeDef *htim);

}

#endif  