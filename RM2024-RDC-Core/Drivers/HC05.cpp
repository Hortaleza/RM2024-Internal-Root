#include "HC05.hpp"

#if USE_HC05
namespace HC05
{

bool has_received = false;
bool connected  = false; // needs updating



uint8_t rxBuffer[DATASIZE] = {};
uint8_t boxesChosen[DATASIZE] = {};
uint8_t temp[2] = {};

uint32_t lastReceiveTick = HAL_GetTick();

void clearMemory()
{

}

/**
 * Put this function in the while loop. This function updates a boolean global
 * variable RC16::connected constantly and call clearMemory when not connected
 */
bool getConnectionStatus(uint32_t timeLimit = 100)
{
    if (HAL_GetTick() - lastReceiveTick > timeLimit)
    {
        connected = false;
    }
    else
    {
        connected = true;
    }
    if (connected == false)
        clearMemory();
    return connected;
}

// Error Callback Function
void ErrorCallback(UART_HandleTypeDef *huart)
{
    clearMemory();
    HAL_UART_Abort_IT(huart);
    HAL_UARTEx_ReceiveToIdle_IT(huart, rxBuffer, DATASIZE);
}

// Normal Callback Function
void rxEventCallback(UART_HandleTypeDef *huart, uint16_t dataSize)
{
    has_received = true;

    // Verifiy data validity
    if (dataSize != DATASIZE)
    {
        ErrorCallback(huart);
        return;
    }
    // Record time when receive
    lastReceiveTick = HAL_GetTick();

    // @todo: Decode the data after receiving
for (int i=0;i<DATASIZE;i++){
temp[i] = rxBuffer[i];
//boxesChosen[i] = rxBuffer[i];
}
if (temp[1]>temp[0]){
    boxesChosen[0] = temp[0];
    boxesChosen[1] = temp[1];
}
else if (temp[1]<temp[0]){
    boxesChosen[0] = temp[1];
    boxesChosen[1] = temp[0];
}

    // @todo: Verify decoded data range;

    bool valid = 1;
    if (!valid)
    {
        ErrorCallback(huart);
        return;
    }



    // Repeat
    HAL_UARTEx_ReceiveToIdle_IT(huart, rxBuffer, DATASIZE);
}

/*================================================================================*/
void init()
{
    HAL_UART_RegisterRxEventCallback(&huart3, rxEventCallback);
    HAL_UART_RegisterCallback(&huart3, HAL_UART_ERROR_CB_ID, ErrorCallback);
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, rxBuffer, DATASIZE);
}
} // namespace HC06

#endif
