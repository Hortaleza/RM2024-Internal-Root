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
int16_t currentRPM = 0;
/**
 * @todo Show your control outcome of the M3508 motor as follows
 */
float myKp123           = 10;
int16_t targetRPM = 5000;
void motorTask(void *)
{
    DJIMotor::MotorSet motorset;
    // TODO: motorset.setCurrentLimit();
    static Control::PID motorPID(myKp123, 2, 0);
    while (true)
    {
        /* Your user layer codes in loop begin here*/
        /*=================================================*/
        currentRPM   = motorset[0].getRPM();
        static volatile float output;
        output = motorPID.update(targetRPM, currentRPM, 0.001f);
        // Remember when changing dt, change the delay as well
        // target is from the controller through DR16
        motorset[0].setCurrent(output);
        motorset.transmit();  // Transmit the data to the motor // in a package
        
        vTaskDelay(1);  // Delay and block the task for 1ms.
    }
}

void controllerTask(void *)
{
    
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
                      &xControllerTaskTCB)
    /**
     * @todo Add your own task here
     */
}
