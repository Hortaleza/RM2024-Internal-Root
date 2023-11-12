
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

namespace TRControl
{
    
void WholeTRControl(int delay);
void runFastMode(int delay);
void runAccurateMode(int delay);

} // namespace TRControl




#endif