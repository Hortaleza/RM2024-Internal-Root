
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
#ifndef ARM_VERTICAL
#define ARM_VERTICAL 4
#endif
#ifndef ARM_HORIZONTAL
#define ARM_HORIZONTAL 5
#endif

namespace TRControl
{

// PID Parameters
const float Kp = 12;
const float Ki = 0.5;
const float Kd = 45;

const float Kp_arm = 12;
const float Ki_arm = 0.5;
const float Kd_arm = 45;

const float Kp_aut = 12;
const float Ki_aut = 0.5;
const float Kd_aut = 45;

const float ARMSpeed = 0.3;

void wholeTRControl(int delay);
void runNormalMode(float speed, int delay);
void runArmMode(float speed, int delay);
void runAutoMode(int delay);

} // namespace TRControl




#endif