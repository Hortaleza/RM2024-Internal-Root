#pragma once
#include "AppConfig.h"

#if IS_AR

#include "main.h"
#include "PID.hpp"
#include "DJIMotor.hpp"
namespace ARControl
{
void updateStatus();
void forward();
void left();
void right();
void stop();
void run();
}
#endif