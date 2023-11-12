#include "DJIMotor.hpp"

// DEF
#if USE_DJI_MOTOR
#ifndef RDC_DJIMotor_MAX_NUM
#define RDC_DJIMotor_MAX_NUM 8
#endif
#include "can.h"
namespace DJIMotor
{

uint8_t rxData[8] = {};

void seperateIntoTwoBytes(const int16_t &original,
                          int8_t &higher,
                          int8_t &lower)
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

    // getFilter();
}

void DJIMotor::setOutput(int16_t output)
{
    int8_t higher, lower;
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
    if (current > MAX_CURRENT - 1)
    {
        setOutput(MAX_SIZE);
        return -1;
    }
    if (current < -MAX_CURRENT + 1)
    {
        setOutput(-MAX_SIZE);
        return -1;
    }
    setOutput(int16_t(((current) / MAX_CURRENT) * (MAX_SIZE - 1)));
    return 0;
}

int16_t DJIMotor::getRPM()
{
    return rpm;
}

uint8_t DJIMotor::getTemperature()
{
    return temperature;
}

int16_t DJIMotor::getCurrent()
{
    return actualCurrent;
}

// void DJIMotor::getFilter()
// {
//     // TODO: Discuss if this function is necessary
//     uint32_t filter_id = 0x200 + canID;
//     uint32_t filter_mask = 0x7FF;
//     // filter            = {0,
//     //                      filterID,
//     //                      0,
//     //                      0,
//     //                      CAN_FILTER_FIFO0,
//     //                      ENABLE,
//     //                      CAN_FILTERMODE_IDMASK,
//     //                      CAN_FILTERSCALE_16BIT,
//     //                      CAN_FILTER_ENABLE,
//     //                      0};
//     CAN_FilterTypeDef local_filter = {filter_id << 5,
//                                 0,
//                                 0, 
//                                 0,
//                                 CAN_FILTER_FIFO0,
//                                 ENABLE,
//                                 CAN_FILTERMODE_IDMASK,
//                                 CAN_FILTERSCALE_16BIT,
//                                 CAN_FILTER_ENABLE,
//                                 0};
//     // CAN_FilterTypeDef local_filter = {((filter_id << 5)  | (filter_id >> (32
//     // - 5))) & 0xFFFF,
//     //                             (filter_id >> (11 - 3)) & 0xFFF8,
//     //                             ((filter_mask << 5)  | (filter_mask >> (32 -
//     //                             5))) & 0xFFFF, (filter_mask >> (11 - 3)) &
//     //                             0xFFF8,
//     //                              CAN_FILTER_FIFO0,
//     //                             ENABLE,
//     //                             CAN_FILTERMODE_IDMASK,
//     //                             CAN_FILTERSCALE_16BIT,
//     //                             CAN_FILTER_ENABLE,
//     //                             0};
//     filter = local_filter;
//     // filter.FilterIdHigh = ((filter_id << 5)  | (filter_id >> (32 - 5))) & 0xFFFF; // STID[10:0] & EXTID[17:13]
//     // filter.FilterIdLow = (filter_id >> (11 - 3)) & 0xFFF8; // EXID[12:5] & 3 Reserved bits
//     // filter.FilterMaskIdHigh = ((filter_mask << 5)  | (filter_mask >> (32 - 5))) & 0xFFFF;
//     // filter.FilterMaskIdLow = (filter_mask >> (11 - 3)) & 0xFFF8;

//     // filter.FilterFIFOAssignment = CAN_FilterFIFO0;
//     // filter.FilterNumber = filter_num;
//     // filter.FilterMode = CAN_FilterMode_IdMask;
//     // filter.FilterScale = CAN_FilterScale_32bit;
//     // filter.FilterActivation = ENABLE;
//     //CAN_FilterInit(&filter);
// }

void DJIMotor::update()
{
    // CAN is stable so no need to check validity

    position      = rxData[1] | (rxData[0] << 8);
    rpm           = rxData[3] | (rxData[2] << 8);
    actualCurrent = rxData[5] | (rxData[4] << 8);
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
uint32_t mailbox1, mailbox2;
CAN_TxHeaderTypeDef txHeader1 = {
    0x200, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
CAN_TxHeaderTypeDef txHeader2 = {
    0x1FF, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
void MotorSet::transmit()
{
    HAL_CAN_AddTxMessage(&hcan, &txHeader1, txData1, &mailbox1);
    HAL_CAN_AddTxMessage(&hcan, &txHeader2, txData2, &mailbox2);
    // TODO: Return Status Code
}

void receiveTaskInit()
{
    CAN_FilterTypeDef filter = {0x200 << 5,
                                0,
                                0,
                                0,
                                CAN_FILTER_FIFO0,
                                ENABLE,
                                CAN_FILTERMODE_IDMASK,
                                CAN_FILTERSCALE_32BIT,
                                CAN_FILTER_ENABLE,
                                0};
    HAL_CAN_ConfigFilter(&hcan, &filter);
}

void receiveTaskLoop(CAN_RxHeaderTypeDef *rxheader, MotorSet& ms)
{
    while (HAL_CAN_GetRxMessage(
               &hcan, CAN_RX_FIFO0, rxheader, rxData) != HAL_OK)
        ;
    int canID = (*rxheader).StdId - 0x200;
    if (!(canID > 0 && canID < 9))
        return;
    ms[canID - 1].update();
}

void init() { receiveTaskInit(); }

MotorSet motorset = MotorSet();




}
// namespace DJIMotor
#endif