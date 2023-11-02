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

uint16_t currentToOutput(float current);
uint16_t concatenateTwoBytes(const uint8_t &higher, const uint8_t &lower);
void seperateIntoTwoBytes(const uint16_t &original,
                          uint8_t &higher,
                          uint8_t &lower);
const uint16_t MAX_SIZE = 16384;
const float MAX_CURRENT = 20.0f;
class DJIMotor
{
   public:
    uint8_t canID;          // NOTE THAT canID = index + 1 !!!!!!!!
    uint8_t *txData1;
    uint8_t *txData2;
    
    void init(uint8_t canID, uint8_t *txData1, uint8_t *txData2);

    void setOutput(uint16_t output);
    int setCurrent(float current);
    int16_t getRPM();
    uint8_t getTemperature();
    int16_t getCurrent();
    void operator=(uint16_t output) { setOutput(output); }
    int operator<<(float current) {
        return setCurrent(current);
    }

   private:
    CAN_FilterTypeDef filter;
    float position;
    int16_t rpm;
    int16_t actualCurrent;
    //  int16_t setCurrent;
    //  uint16_t currentLimit;

    uint8_t temperature;

    //  int32_t rotaryCnt;
    //  int16_t positionOffset;

    //  uint32_t disconnectCnt;
    //  uint32_t receiveCnt;
    bool connected;
    void getFilter();
    void update();
};

class MotorSet
{
   private:
    DJIMotor motors[8];


   public:
    uint8_t txData1[8] = {};
    uint8_t txData2[8] = {};
    MotorSet();
    DJIMotor &operator[](int i) { return motors[i]; }
    void transmit();
    void setCurrentLimit(float limit); // TODO
};

/**
 * @brief A motor's handle. We do not require you to master the cpp class
 * syntax.
 * @brief However, some neccessary OOP thought should be shown in your code.
 * @brief For example, if you have multiple motors, which is going to happen in
 * RDC (You have at least 4 wheels to control)
 * @brief You are able to write a "template" module for all the abstract motors,
 * and instantiate them with different parameters
 * @brief Instead of copy and paste your codes for four times
 * @brief This is what we really appreiciate in our programming
 * 
 * 
 */

void init();


/*===========================================================*/
/**
 * @brief You can define your customized function here
 * @note  It might not be necessary in your PA3, but it's might be beneficial
for your RDC development progress
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