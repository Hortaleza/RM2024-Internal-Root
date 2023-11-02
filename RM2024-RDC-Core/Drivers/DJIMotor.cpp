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

int DJIMotor::setOutput(uint16_t output)
{
    bool normal = true;
    // Check output range
    if (output > 16384)
    {
        normal = false;
        output = 16384;
    }
    if (output < -16384)
    {
        normal = false;
        output = -16384;
    }

    // Change txData1/2
    // return -1 if the input is out of range or when any other special cases
    // happen
    uint8_t higher, lower;
    seperateIntoTwoBytes(output, higher, lower);
    do{
        if (canID < 5 && canID > 0)
        {
            txData1[canID * 2 - 2] = higher;
            txData1[canID * 2 - 1] = lower;
            break;
        }
        if (canID >= 5 && canID < 9)
        {
            txData1[canID * 2 - 10] = higher;
            txData1[canID * 2 - 9] = lower;
            break;
        }
        normal = false;
        break;
    } while (1);

    if (normal)
        return 0;
    else return -1;
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
        this->motors[i].init(i + 1, txData1, txData2);
    }
}
void MotorSet::transmit()
{
    uint32_t mailbox;
    // TODO: Transmit twice for 2 data buffers
    CAN_TxHeaderTypeDef txHeader = {
        0x1FF, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
    HAL_CAN_AddTxMessage(&hcan, &txHeader, txData1, &mailbox);
    HAL_CAN_AddTxMessage(&hcan, &txHeader, txData2, &mailbox);
    // TODO: Return Status Code
}

uint16_t currentToOutput(float current)
{
    // There may be problems due to the data range (*16383 may be too big (actually should times 16384))
    return uint32_t(((current) / MAX_CURRENT) * 16383);
}

// Initialize motor's controller instance
void init()
{
    HAL_CAN_Start(&hcan);
}

}
// namespace DJIMotor
#endif