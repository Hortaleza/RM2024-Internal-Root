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
#include "HCSR04.hpp" 
// #include "MG996R.hpp"
#include "TRControlTask.hpp"
#include "ARControlTask.hpp"


/*Allocate the stack for our PID task*/
StackType_t uxPIDTaskStack[256];
StackType_t uxReceiveTaskStack[256];
StackType_t uxARTaskStack[256];
StackType_t uxUltraSoundTaskStack[256];

/*Declare the PCB for our PID task*/
StaticTask_t xPIDTaskTCB;
StaticTask_t xReceiveTaskTCB;
StaticTask_t xARTaskTCB;
StaticTask_t xUltraSoundTaskTCB;

uint16_t distance = 0;

/**
 * @todo Show your control outcome of the M3508 motor as follows
 */
void motorTask(void *)
{
    // TODO: motorset.setCurrentLimit();
    int delay = 1;
    while (true)
    {
        TRControl::WholeTRControl(delay);
        vTaskDelay(delay);  // Delay and block the task for 1ms.
    }
}

void ultraSoundTask(void *)
{
    // HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
    while (true)
    {
        distance = HCSR04::HCSR04_Read();
        vTaskDelay(1);  // Delay and block the task for 1ms.
    }
}

void CANReceiveTask(void *)
{
    DJIMotor::receiveTaskInit();
    CAN_RxHeaderTypeDef rxheader;

    while (true)
    {
        DJIMotor::receiveTaskLoop(&rxheader, DJIMotor::motorset);
        vTaskDelay(1);
    }
}

void ARTask(void *)
{
    while (true)
    {
        // For test use

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
    xTaskCreateStatic(
        motorTask, "motorTask", 256, NULL, 1, uxPIDTaskStack, &xPIDTaskTCB);
    xTaskCreateStatic(CANReceiveTask,
                      "CANReceiveTask",
                      256,
                      NULL,
                      1,
                      uxReceiveTaskStack,
                      &xReceiveTaskTCB);
    xTaskCreateStatic(
        ARTask, "ARTask", 256, NULL, 1, uxARTaskStack, &xARTaskTCB);
    xTaskCreateStatic(
        ultraSoundTask,
        "ultraSoundTask",
        256,
        NULL,
        1,
        uxUltraSoundTaskStack,
        &xUltraSoundTaskTCB);  // Add the main task into the scheduler

    /**
     * @todo Add your own task here
     */
}
