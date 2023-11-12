#include "LineTracker.hpp"

#if USE_LINETRACKER

namespace LineTracker
{

uint16_t A0PIN = GPIO_PIN_6;
uint16_t D0PIN = GPIO_PIN_7;

static volatile bool status = 0;

void updateStatus()
{
    status = HAL_GPIO_ReadPin(GPIOA, A0PIN);
}

}
#endif