#pragma once
#include "AppConfig.h"

#if IS_AR

#include "main.h"
#include "PID.hpp"
#include "DJIMotor.hpp"
namespace ARControl
{
void updateStatus();
void forward(int);
void left(int);
void right(int);
void stop();
void run();
void test();
void turnFixedDistance(uint32_t, int);
void turn (uint32_t);
}
#endif