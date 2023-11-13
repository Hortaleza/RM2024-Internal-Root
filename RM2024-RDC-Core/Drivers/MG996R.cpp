#include "MG996R.hpp"
#if USE_MG996R

namespace MG996R
{
  #include "gpio.h"
  #include "tim.h"
  #include "stm32f1xx_hal.h"



  void setServoAngle(uint16_t angle) {
  // Map the angle to pulse width
  uint16_t pulseWidth;
  //0=1500
  //180=7500

  pulseWidth = (2.5+(angle/18))*20000/100;

  // depending on tim1 period in tim.c
  // Set the pulse width for the PWM signal
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulseWidth);
  }


}

#endif