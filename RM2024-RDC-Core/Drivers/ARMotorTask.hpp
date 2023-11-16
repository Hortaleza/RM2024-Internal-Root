#pragma once
#include "AppConfig.h"

#if IS_AR

#include "main.h"
#include "PID.hpp"
#include "DJIMotor.hpp"
namespace ARMotor
{
     void forward(int RPM);
     void left(int RPM);
     void right(int RPM);
     void stop();
     void MotorTask();

}
#endif