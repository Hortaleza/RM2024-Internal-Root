#include "MG996R.hpp"
#if USE_MG996R

namespace MG996R
{
    #include "gpio.h"S
    #include "tim.h"
    #include "stm32f1xx_hal.h"

    void PWM_Init(){

    }

    void setServoAngle(double angle) {
    // Map the angle to pulse width
    uint16_t pulseWidth;

    //0=1500
    //180=7500

    pulseWidth = (2.5+(angle/18))*60000/100;

    // depending on tim1 period in tim.c
    // Set the pulse width for the PWM signal
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulseWidth);
    }



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
}

#endif