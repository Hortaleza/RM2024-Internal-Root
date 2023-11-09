#include "DJIMotor.hpp"

// DEF
#ifdef USE_DJI_MOTOR
#ifndef RDC_DJIMotor_MAX_NUM
#define RDC_DJIMotor_MAX_NUM 8
#endif

/*
 * This is actually the driver for C620 controller, not just DJIMotors.
 * However, changing the name of the file means to modify the CMake file which I am lazy to do.
 */


namespace DJIMotor
{

uint32_t mailbox0, mailbox1;
CAN_TxHeaderTypeDef txHeader0 = {
    0x200, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
CAN_TxHeaderTypeDef txHeader1 = {
    0x1FF, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};

uint8_t txData0[8] = {};
uint8_t txData1[8] = {};

CAN_FilterTypeDef rxFilter = {0x200,
                              0,
                              0x3f0,
                              0,
                              CAN_FILTER_FIFO0,
                              ENABLE,
                              CAN_FILTERMODE_IDMASK,
                              CAN_FILTERSCALE_16BIT,
                              CAN_FILTER_ENABLE,
                              0};

CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8] = {};

DJIMotor Motors[8];

void DJIMotor::update()
{
    // Update the data from rxData WITHOUT any check!
    rawPosition   = rxData[1] | (rxData[0] << 8);
    rpm           = rxData[3] | (rxData[2] << 8);
    actualCurrent = rxData[5] | (rxData[4] << 8);
    temperature   = rxData[6];
}

void DJIMotor::setOutput(int16_t output)
{
    int8_t higher, lower;

    // Seperate into 2 bytes
    higher = (output >> 8) & 0b11111111;
    lower  = output & 0b11111111;

    // Distribute data according to canID
    if (canID < 5 && canID > 0)
    {
        txData0[canID * 2 - 2] = higher;
        txData0[canID * 2 - 1] = lower;
        return;
    }
    if (canID >= 5 && canID < 9)
    {
        txData1[canID * 2 - 10] = higher;
        txData1[canID * 2 - 9]  = lower;
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

DJIMotor &getMotorByID(uint8_t canID) { return Motors[canID - 1]; }

// void ErrorCallback(__CAN_HandleTypeDef *hcan)
// {
//     // Receive again
//     HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);
// }

void rxCallback(__CAN_HandleTypeDef *hcan)
{
    // Check rxHeader and Update the corresponding motor's information
    uint8_t receivedCanID = (rxHeader.StdId >> 5) - 0x200;
    // If reads the sent message, read again (Don't know if this is useful??
    if (receivedCanID == 0)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);
        return;
    }

    // TODO: Check validity of the data

    // Update data
    getMotorByID(receivedCanID).update();
}

void transmit()
{
    HAL_CAN_AddTxMessage(&hcan, &txHeader0, txData0, &mailbox0);
    HAL_CAN_AddTxMessage(&hcan, &txHeader1, txData1, &mailbox1);
}

void init()
{
    // Distribute canID
    for (int i = 0; i < 8; i++) {
        Motors[i].canID = i + 1;
    }
    HAL_CAN_ConfigFilter(&hcan, &rxFilter);
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID);
    HAL_CAN_RegisterCallback(
        &hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, rxCallback);
    HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData);
    // HAL_CAN_ActivateNotification(&hcan, CAN_IT_ERROR);
}


} // End of namespace DJIMotor
#endif
