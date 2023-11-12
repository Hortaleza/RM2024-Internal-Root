
#pragma once
#include "AppConfig.h"

#if IS_TR
#include "DJIMotor.hpp"
#include "DR16.hpp"
#include "PID.hpp"

namespace TRControl
{

void runFastMode(int delay);

} // namespace TRControl




#endif