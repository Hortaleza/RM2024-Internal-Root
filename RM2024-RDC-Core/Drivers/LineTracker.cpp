#include "LineTracker.hpp"

// namespace LineTracker{

//     struct sensor{

//         uint16_t gpio;

//     };

//     CAN_TxHeaderTypeDef txHeaderAR = {0x200, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
//     uint8_t txDataAR[8] = {};
//     uint32_t mailbox;

//     uint8_t readButton(uint16_t gpio){

//         return HAL_GPIO_ReadPin(GPIOA, gpio);

//     }

//     void moveForward(){

//         //PID 
//         txDataAR = //PID result
//         HAL_CAN_AddTxMessage(&hcan, &txHeaderAR, txDataAR, &mailbox);

//     }

// }