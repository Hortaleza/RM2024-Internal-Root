#include "DJIMotor.hpp"

// DEF
#ifdef USE_DJI_MOTOR
#ifndef RDC_DJIMotor_MAX_NUM
#define RDC_DJIMotor_MAX_NUM 8
#endif
#include "can.h"
namespace DJIMotor
{
uint16_t concatenateTwoBytes(const uint8_t &higher, const uint8_t &lower)
{
    return lower | (higher << 8);
}

void seperateIntoTwoBytes(const uint16_t &original,
                          uint8_t &higher,
                          uint8_t &lower)
{
    higher = (original >> 8) & 0b11111111;
    lower  = original & 0b11111111;
}

void DJIMotor::init(uint8_t id, uint8_t *t1, uint8_t *t2)
{
    // Not doing it in the constructor is to avoid dynamic memory distribution
    // Need to set canID in advance
    canID   = id;
    txData1 = t1;
    txData2 = t2;
    getFilter();
    update();
}

void DJIMotor::setOutput(uint16_t output)
{
    uint8_t higher, lower;
    seperateIntoTwoBytes(output, higher, lower);
    if (canID < 5 && canID > 0)
    {
        txData1[canID * 2 - 2] = higher;
        txData1[canID * 2 - 1] = lower;
        return;
    }
    if (canID >= 5 && canID < 9)
    {
        txData2[canID * 2 - 10] = higher;
        txData2[canID * 2 - 9]  = lower;
        return;
    }
    // Do nothing if no cases fit
}

int DJIMotor::setCurrent(float current)
{
    if (current > MAX_CURRENT - 0.001)
    {
        setOutput(MAX_SIZE);
        return -1;
    }
    if (current < -MAX_CURRENT + 0.001)
    {
        setOutput(-MAX_SIZE);
        return -1;
    }
    setOutput(uint32_t(((current) / MAX_CURRENT) * (MAX_SIZE - 1)));
    return 0;
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
    CAN_FilterTypeDef local_filter = {((filter_id << 5)  | (filter_id >> (32 - 5))) & 0xFFFF, 
                                (filter_id >> (11 - 3)) & 0xFFF8,
                                ((filter_mask << 5)  | (filter_mask >> (32 - 5))) & 0xFFFF,
                                (filter_mask >> (11 - 3)) & 0xFFF8,
                                 CAN_FILTER_FIFO0, 
                                ENABLE, 
                                CAN_FILTERMODE_IDMASK,
                                CAN_FILTERSCALE_16BIT,
                                CAN_FILTER_ENABLE,
                                0};
    filter = local_filter;
    // filter.FilterIdHigh = ((filter_id << 5)  | (filter_id >> (32 - 5))) & 0xFFFF; // STID[10:0] & EXTID[17:13]
    // filter.FilterIdLow = (filter_id >> (11 - 3)) & 0xFFF8; // EXID[12:5] & 3 Reserved bits
    // filter.FilterMaskIdHigh = ((filter_mask << 5)  | (filter_mask >> (32 - 5))) & 0xFFFF;
    // filter.FilterMaskIdLow = (filter_mask >> (11 - 3)) & 0xFFF8;

    // filter.FilterFIFOAssignment = CAN_FilterFIFO0;
    // filter.FilterNumber = filter_num;
    // filter.FilterMode = CAN_FilterMode_IdMask;
    // filter.FilterScale = CAN_FilterScale_32bit;
    // filter.FilterActivation = ENABLE;
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

    position      = concatenateTwoBytes(rxData[0], rxData[1]);
    rpm           = concatenateTwoBytes(rxData[2], rxData[3]);
    actualCurrent = concatenateTwoBytes(rxData[4], rxData[5]);
    temperature   = rxData[6];
}

MotorSet::MotorSet()
{
    // Distribute canID & txData pointer
    for (int i = 0; i < 8; i++)
    {
        this->motors[i].init(i + 1, txData1, txData2);
    }
}
void MotorSet::transmit()
{
    uint32_t mailbox1, mailbox2;
    CAN_TxHeaderTypeDef txHeader1 = {
        0x200, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
    CAN_TxHeaderTypeDef txHeader2 = {
        0x1FF, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
    HAL_CAN_AddTxMessage(&hcan, &txHeader1, txData1, &mailbox1);
    HAL_CAN_AddTxMessage(&hcan, &txHeader2, txData2, &mailbox2);
    // TODO: Return Status Code
}

// Initialize motor's controller instance
void init()
{
    HAL_CAN_Start(&hcan);
}

}
// namespace DJIMotor
#endif