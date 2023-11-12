#pragma once
#include "AppConfig.h"

#if USE_MG996R

namespace MG996R
{
#include "stm32f1xx_hal.h"

void Error_Handler(void);

void setServoAngle(double angle);

}
#endif

