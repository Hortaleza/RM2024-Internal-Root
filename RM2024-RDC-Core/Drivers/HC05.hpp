#pragma once
#include "AppConfig.h"

#if USE_HC05

#include "main.h"
#include "stdint.h"
#include "usart.h"

namespace HC05
{

//extern bool boxesChosen[4];


const int DATASIZE = 2;
extern uint8_t boxesChosen[DATASIZE];

void init();
bool getConnectionStatus(uint32_t timeLimit);

} // namespace HC05


#endif