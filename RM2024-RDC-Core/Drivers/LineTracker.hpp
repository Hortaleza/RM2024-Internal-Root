#pragma once

#include "RM2024-Internal-Root\Core\Inc\main.h"
#include DJIMotor.hpp

/**
 * @brief read the button input
 *
 * @return uint8_t 0 if button is pressed, 1 if not
 */

namespace LineTracker{

    struct sensor;

    uint8_t readButton(uint16_t gpio);

    void moveForward();

}
