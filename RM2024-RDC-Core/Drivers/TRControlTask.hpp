
#pragma once
#include "AppConfig.h"

#if IS_TR
#include "DJIMotor.hpp"
#include "DR16.hpp"
#include "PID.hpp"

#ifndef FR
#define FR 0
#endif
#ifndef FL
#define FL 1
#endif
#ifndef BL
#define BL 2
#endif
#ifndef BR
#define BR 3
#endif
#ifndef ARM
#define ARM 4
#endif

namespace TRControl
{
    
void WholeTRControl(int delay);
void runNormalMode(float speed, int delay);
void runArmMode(float speed, int delay);
void runAutoMode(int delay);

} // namespace TRControl




#endif