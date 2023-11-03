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
StackType_t uxControllerTaskStack[256];
/*Declare the PCB for our PID task*/
StaticTask_t xPIDTaskTCB;
StaticTask_t xControllerTaskTCB;
const int16_t MAX_RPM = 19000;
double uniformed[8]   = {};
int16_t currentRPM[8] = {};
int16_t targetRPM[8]  = {};
/**
 * @todo Show your control outcome of the M3508 motor as follows
 */
void motorTask(void *)
{
    DJIMotor::MotorSet motorset;
    // TODO: motorset.setCurrentLimit();
    static Control::PID motorPID0(10, 2, 0);
    static Control::PID motorPID1(10, 2, 0);
    while (true)
    {
        targetRPM[0]  = (int16_t)(uniformed[0] * MAX_RPM);
        currentRPM[0] = motorset[0].getRPM();
        motorset[0].setCurrent(
            motorPID0.update(targetRPM[0], currentRPM[0], 0.001f));
        targetRPM[1]  = (int16_t)(uniformed[1] * MAX_RPM);
        currentRPM[1] = motorset[1].getRPM();
        motorset[1].setCurrent(
            motorPID1.update(targetRPM[1], currentRPM[1], 0.001f));

        motorset.transmit();  // Transmit the data to the motor in a package
        
        vTaskDelay(1);  // Delay and block the task for 1ms.
    }
}

void controllerTask(void *)
{
    const DR16::RcData* RcData = DR16::getRcData();

    while (true)
    {
        uniformed[0] = 2 * (double(RcData->channel1) - DR16::RANGE_DEFAULT) /
                    (DR16::RANGE_MAX - DR16::RANGE_MIN);
        uniformed[1] = 2 * (double(RcData->channel3) - DR16::RANGE_DEFAULT) /
                       (DR16::RANGE_MAX - DR16::RANGE_MIN);
        vTaskDelay(1);  // Delay and block the task for 1ms.
    }
}

/**
 * @brief Intialize all the drivers and add task to the scheduler
 * @todo  Add your own task in this file
*/

void startUserTasks()
{
    // DJIMotor::init();  // Initalize the DJIMotor driver
    
    DR16::init();      // Intialize the DR16 driver

    xTaskCreateStatic(motorTask,
                      "motorTask",
                      256,
                      NULL,
                      1,
                      uxPIDTaskStack,
                      &xPIDTaskTCB);  // Add the main task into the scheduler
    xTaskCreateStatic(controllerTask,
                      "controllerTask",
                      256,
                      NULL,
                      1,
                      uxControllerTaskStack,
                      &xControllerTaskTCB);
    /**
     * @todo Add your own task here
     */
}
