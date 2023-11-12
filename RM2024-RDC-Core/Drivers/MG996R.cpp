#include "MG996R.hpp"
#if USE_MG996R

namespace MG996R
{
#include "gpio.h"
#include "tim.h"


  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();
<<<<<<< HEAD
=======
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  setServoAngle(0); //Requires angle
  HAL_Delay(1000);
}
>>>>>>> f8e3da21706812218ea961ea9f277e6416c4796d

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


}

#endif