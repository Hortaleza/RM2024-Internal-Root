#pragma once
#include "AppConfig.h"

#if USE_MG996R

namespace MG996R
{
  #include "stm32f1xx_hal.h"


  void setServoAngle(uint16_t angle);


}
#endif
