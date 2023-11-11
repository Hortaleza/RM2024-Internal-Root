#pragma once
#include "AppConfig.h"

#if USE_MG996R

namespace MG996R
{
#include "stm32f1xx_hal.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void Error_Handler(void);

void setServoAngle(double angle);

}
#endif

