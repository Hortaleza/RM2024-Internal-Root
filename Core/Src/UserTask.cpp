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
/*Declare the PCB for our PID task*/
StaticTask_t xPIDTaskTCB;
int16_t currentRPM = 0;
/**
 * @todo Show your control outcome of the M3508 motor as follows
 */
int16_t targetRPM = 3;
void userTask(void *)
{
    DJIMotor::MotorSet motorset;
    // TODO: motorset.setCurrentLimit();
    motorset[0] << 0.1;
    vTaskDelay(10000);
    while (true)
    {
        /* Your user layer codes in loop begin here*/
        /*=================================================*/
        
        // currentRPM = motorset[0].getRPM();
        // static Control::PID motorPID(0, 0, 0);

        // float output = motorPID.update(targetRPM, currentRPM, 0.001f);
        // // Remember when changing dt, change the delay as well
        // // target is from the controller through DR16

        // motorset[0].setOutput(output);
        // motorset.transmit();  // Transmit the data to the motor // in a package
        motorset[0] = 0;
        vTaskDelay(1);  // Delay and block the task for 1ms.
    }
}

/**
 * @todo In case you like it, please implement your own tasks
 */


/**
 * @brief Intialize all the drivers and add task to the scheduler
 * @todo  Add your own task in this file
*/
void startUserTasks()
{
    DJIMotor::init();  // Initalize the DJIMotor driver
    
    DR16::init();      // Intialize the DR16 driver

    xTaskCreateStatic(userTask,
                      "user_default ",
                      256,
                      NULL,
                      1,
                      uxPIDTaskStack,
                      &xPIDTaskTCB);  // Add the main task into the scheduler
    /**
     * @todo Add your own task here
    */
}
