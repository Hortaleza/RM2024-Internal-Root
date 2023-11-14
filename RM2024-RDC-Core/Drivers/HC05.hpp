#pragma once
#include "AppConfig.h"

#if USE_HC05

#include "main.h"
#include "stdint.h"
#include "usart.h"

namespace HC05
{

const int DATASIZE = 1;
void init();
bool getConnectionStatus(uint32_t timeLimit);
} // namespace HC05


#endif