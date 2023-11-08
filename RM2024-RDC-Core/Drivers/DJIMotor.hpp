/**
 * @file DJIMotor.hpp
 * @author - GUO, Zilin
 *         - Your Name
 * @brief This is the DJIMotor template codes for RM2024-Tutorial PA3 and RDC
 * @note  You could directly transplant your code to the RDC after finishing PA3
 * @note  If you do not like the template I provide for you, you could remove
 * all of them and use your own
 * @copyright This file is only for HKUST Enterprize RM2024 internal
 * competition. All Rights Reserved.
 *
 */

#pragma once
#include "AppConfig.h"

#if USE_DJI_MOTOR

#ifndef DJI_MOTOR_CAN
#define DJI_MOTOR_CAN hcan
#endif

#include "main.h"
#include "can.h"
namespace DJIMotor
{
const uint16_t MAX_SIZE = 16384;
const float MAX_CURRENT = 20000.0f;

class DJIMotor
{
   public:
    uint8_t canID;
    int16_t rawPosition;
    int16_t rpm;
    int16_t actualCurrent;
    uint8_t temperature;

    // TODO: Calculate the accumulated position!

    void update();
    void setOutput(int16_t output);
    int setCurrent(float current);
};
extern DJIMotor Motors[8];

DJIMotor &getMotorByID(uint8_t canID);
void transmit();
void init();
/**
 * @brief A motor's handle. We do not require you to master the cpp class
 * syntax.
 * @brief However, some neccessary OOP thought should be shown in your code.
 * @brief For example, if you have multiple motors, which is going to happen
 * in RDC (You have at least 4 wheels to control)
 * @brief You are able to write a "template" module for all the abstract
 * motors, and instantiate them with different parameters
 * @brief Instead of copy and paste your codes for four times
 * @brief This is what we really appreiciate in our programming
 *
 *
 */

/*===========================================================*/
/**
 * @brief You can define your customized function here
 * @note  It might not be necessary in your PA3, but it's might be
beneficial for your RDC development progress
 * @example
 * float get(uint16_t canID);
 *
 * @note You could try to normalize the encoder's value and work out the
accumulated position(orientation) of the motor
 * float getPosition(uint16_t canID);
 * ..... And more .....
 *
============================================================*/

/*===========================================================*/
}  // namespace DJIMotor
#endif  // USE_DJI_MOTOR