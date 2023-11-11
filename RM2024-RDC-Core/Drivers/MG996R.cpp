#include "MG996R.hpp"
#if USE_MG996R

namespace MG996R
{
#include "gpio.h"
#include "tim.h"


  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();

void setServoAngle(double angle) {
  // Map the angle to pulse width
  uint16_t pulseWidth;

  if (angle <= 90.0) {
    // Clockwise rotation
    pulseWidth = (uint16_t)((angle * 0.01) + 0.5) * 1000;
  } else {
    // Anticlockwise rotation
    pulseWidth = (uint16_t)(((angle - 90.0) * 0.01) + 1.6) * 1000;
  }

  // Set the pulse width for the PWM signal
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulseWidth);
}


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

}

#endif