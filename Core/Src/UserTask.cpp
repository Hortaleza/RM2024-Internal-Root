/**
 * @file UserTask.cpp
 * @author JIANG Yicheng  RM2024 (EthenJ@outlook.sg)
 * @author GUO, Zilin
 * @brief RDC user task files
 * @version 0.3
 * @date 2022-10-20
 *
 * @copyright Copyright (c) 2024
 */
#include "AppConfig.h"   // Include our customized configuration
#include "DJIMotor.hpp"  // Include DR16
#include "DR16.hpp"
#include "FreeRTOS.h"  // Include FreeRTOS.h
#include "PID.hpp"     // Include PID
#include "main.h"
#include "task.h"  // Include task
#include "TRControlTask.hpp"

/*Allocate the stack for our PID task*/
StackType_t uxPIDTaskStack[256];
StackType_t uxReceiveTaskStack[256];
StackType_t uxDR16TaskStack[256];

/*Declare the PCB for our PID task*/
StaticTask_t xPIDTaskTCB;
StaticTask_t xReceiveTaskTCB;
StaticTask_t xDR16TaskTCB;

/**
 * @todo Show your control outcome of the M3508 motor as follows
 */
void motorTask(void *)
{
    // TODO: motorset.setCurrentLimit();
    while (true)
    {
        TRControl::runFastMode(1);
        vTaskDelay(1);  // Delay and block the task for 1ms.
    }
}


void receiveTask(void *)
{
    DJIMotor::receiveTaskInit();
    CAN_RxHeaderTypeDef rxheader;

    while (true)
    {
        DJIMotor::receiveTaskLoop(&rxheader, DJIMotor::motorset);
        vTaskDelay(1);
    }
}

void DR16Task(void *)
{
    while (true)
    {
        // Update connection status every 100ms
        DR16::getConnectionStatus(100);
        vTaskDelay(10);
    }
}

/**
 * @brief Intialize all the drivers and add task to the scheduler
 * @todo  Add your own task in this file
*/

void startUserTasks()
{
    HAL_CAN_Start(&hcan);

    DJIMotor::init();  // Initalize the DJIMotor driver
    
    DR16::init();      // Intialize the DR16 driver
    xTaskCreateStatic(motorTask,
                      "motorTask",
                      256,
                      NULL,
                      1,
                      uxPIDTaskStack,
                      &xPIDTaskTCB);  // Add the main task into the scheduler
    xTaskCreateStatic(
        receiveTask,
        "receiveTask",
        256,
        NULL,
        1,
        uxReceiveTaskStack,
        &xReceiveTaskTCB);  // Add the main task into the scheduler
    xTaskCreateStatic(
        DR16Task,
        "DR16Task",
        256,
        NULL,
        1,
        uxDR16TaskStack,
        &xDR16TaskTCB);  // Add the main task into the scheduler

    /**
     * @todo Add your own task here
     */
}
