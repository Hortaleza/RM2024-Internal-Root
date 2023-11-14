#include "HC05.hpp"

#if USE_HC05
namespace HC05
{

bool has_received = false;
bool connected  = false; // needs updating



uint8_t rxBuffer[DATASIZE];

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
    HAL_UARTEx_ReceiveToIdle_IT(huart, rxBuffer, 18);
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

    // @todo: Verify decoded data range;

    bool valid = 1;
    if (!valid)
    {
        ErrorCallback(huart);
        return;
    }



    // Repeat
    HAL_UARTEx_ReceiveToIdle_IT(huart, rxBuffer, 64);
}

/*================================================================================*/
void init()
{
    HAL_UART_RegisterRxEventCallback(&huart3, rxEventCallback);
    HAL_UART_RegisterCallback(&huart3, HAL_UART_ERROR_CB_ID, ErrorCallback);
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, rxBuffer, 64);
}
} // namespace HC06

#endif
