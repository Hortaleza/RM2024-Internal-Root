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

/*Allocate the stack for our PID task*/
StackType_t uxPIDTaskStack[256];
StackType_t uxReceiveTaskStack[256];

/*Declare the PCB for our PID task*/
StaticTask_t xPIDTaskTCB;
StaticTask_t xReceiveTaskTCB;

const int16_t MAX_RPM = 19000;
int16_t currentRPM[8] = {};
int16_t targetRPM[8]  = {};
bool connected        = 0;
DJIMotor::MotorSet motorset;
/**
 * @todo Show your control outcome of the M3508 motor as follows
 */
void motorTask(void *)
{
    // TODO: motorset.setCurrentLimit();
    Control::PID motorPID1(10, 2, 0.02);
    // static Control::PID motorPID1(10, 2, 0);
    while (true)
    {
        // targetRPM[0]  = (connected) ? (int16_t)(uniformed[0] * MAX_RPM) : 0;
        // currentRPM[0] = motorset[0].getRPM();
        // motorset[0].setCurrent(
        //     motorPID0.update(targetRPM[0], currentRPM[0], 0.001f));
        int canid         = 2;
        int index         = canid - 1;

        connected = DR16::getConnectionStatus(100);

        // targetRPM[1]  = (connected) ? (int16_t)(DR16::uniformed.channel0 * MAX_RPM) : 0;
        currentRPM[index] = motorset[index].getRPM();

        motorset[index].setCurrent(
            motorPID1.update(targetRPM[index], currentRPM[index], 0.001f));

        motorset.transmit();  // Transmit the data to the motor in a package
        
        vTaskDelay(1);  // Delay and block the task for 1ms.
    }
}


void receiveTask(void *)
{
    DJIMotor::receiveTaskInit();
    CAN_RxHeaderTypeDef rxheader;

    while (true)
    {
        DJIMotor::receiveTaskLoop(&rxheader, motorset);
        vTaskDelay(1);
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

    /**
     * @todo Add your own task here
     */
}
