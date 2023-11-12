
#pragma once
#include "AppConfig.h"

#if IS_TR
#include "DJIMotor.hpp"
#include "DR16.hpp"
#include "PID.hpp"

namespace TRControl
{
    
void WholeTRControl(int delay);
void runFastMode(int delay);
void runAccurateMode(int delay);

} // namespace TRControl




#endif