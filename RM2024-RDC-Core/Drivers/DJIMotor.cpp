#include "DJIMotor.hpp"

// DEF
#ifdef USE_DJI_MOTOR
#ifndef RDC_DJIMotor_MAX_NUM
#define RDC_DJIMotor_MAX_NUM 8
#endif
#include "can.h"
namespace DJIMotor
{
void DJIMotor::init(uint8_t id, uint8_t *t1, uint8_t *t2)
{
    // Not doing it in the constructor is to avoid dynamic memory distribution
    // Need to set canID in advance
    this->canID   = id;
    this->txData1 = t1;
    this->txData2 = t2;
    getFilter();
    update();
}

void DJIMotor::setOutput(float output)
{
    // Change txData1/2
}
int16_t DJIMotor::getRPM()
{
    update();
    return rpm;
}

uint8_t DJIMotor::getTemperature()
{
    update();
    return temperature;
}
int16_t DJIMotor::getCurrent()
{
    update();
    return actualCurrent;
}
void DJIMotor::getFilter()
{
    // TODO: Discuss if this function is necessary
    uint16_t filterID = (0x200 + canID) << 5;
    filter            = {0,
                         filterID,
                         0,
                         0,
                         CAN_FILTER_FIFO0,
                         ENABLE,
                         CAN_FILTERMODE_IDMASK,
                         CAN_FILTERSCALE_16BIT,
                         CAN_FILTER_ENABLE,
                         0};
}
void DJIMotor::update()
{
    // Get information from CAN
    HAL_CAN_ConfigFilter(&hcan, &filter);
    uint8_t rxData[8] = {};
    CAN_RxHeaderTypeDef rxheader;
    HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxheader, rxData);
    // @todo: Distribute the data to the variables
    // CAN is stable so no need to check validity

    rpm           = rxData[2] << 8 | rxData[3];
    position      = 0;  // TO BE FINISHED
    temperature   = 0;  // TO BE FINISHED
    actualCurrent = 0;  // TO BE FINISHED
}

MotorSet::MotorSet()
{
    // Distribute canID & txData pointer
    for (int i = 0; i < 8; i++)
    {
        this->motors[i].init(i, txData1, txData2);
    }
}
void MotorSet::transmit()
{
    uint32_t mailbox;

    CAN_TxHeaderTypeDef txHeader = {
        0x1FF, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
    HAL_CAN_AddTxMessage(&hcan, &txHeader, txData1, &mailbox);
    HAL_CAN_AddTxMessage(&hcan, &txHeader, txData2, &mailbox);
    // TODO: Return Status Code
}

// Initialize motor's controller instance


/*========================================================*/
// Your implementation of the function, or even your customized function, should
// be implemented here
/*========================================================*/
/**
 * @todo
 */
void init()
{
    HAL_CAN_Start(&hcan);
    MotorSet motorset;
}

}
// namespace DJIMotor
#endif