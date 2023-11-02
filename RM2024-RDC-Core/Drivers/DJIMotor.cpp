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
    uint16_t filter_id = 0x200 + canID;
    uint16_t filter_mask = 0x200 + canID;
    // filter            = {0,
    //                      filterID,
    //                      0,
    //                      0,
    //                      CAN_FILTER_FIFO0,
    //                      ENABLE,
    //                      CAN_FILTERMODE_IDMASK,
    //                      CAN_FILTERSCALE_16BIT,
    //                      CAN_FILTER_ENABLE,
    //                      0};
    
    filter.FilterIdHigh = ((filter_id << 5)  | (filter_id >> (32 - 5))) & 0xFFFF; // STID[10:0] & EXTID[17:13]
    filter.FilterIdLow = (filter_id >> (11 - 3)) & 0xFFF8; // EXID[12:5] & 3 Reserved bits
    filter.FilterMaskIdHigh = ((filter_mask << 5)  | (filter_mask >> (32 - 5))) & 0xFFFF;
    filter.FilterMaskIdLow = (filter_mask >> (11 - 3)) & 0xFFF8;

    filter.FilterFIFOAssignment = CAN_FilterFIFO0;
    filter.FilterNumber = filter_num;
    filter.FilterMode = CAN_FilterMode_IdMask;
    filter.FilterScale = CAN_FilterScale_32bit;
    filter.FilterActivation = ENABLE;
    //CAN_FilterInit(&filter);
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