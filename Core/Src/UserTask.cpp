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
#include "MG996R.hpp"
#include "TRControlTask.hpp"
#include "ARControlTask.hpp"
#include "HC05.hpp"


/*Allocate the stack for our PID task*/
StackType_t uxPIDTaskStack[256];
StackType_t uxReceiveTaskStack[256];
StackType_t uxARTaskStack[512];
StackType_t uxUltraSoundTaskStack[256];
StackType_t uxGearMotorTaskStack[256];
StackType_t uxBTreceiveStack[256];
StackType_t uxARMotorStack[512];
/*Declare the PCB for our PID task*/
StaticTask_t xPIDTaskTCB;
StaticTask_t xReceiveTaskTCB;
StaticTask_t xARTaskTCB;
StaticTask_t xUltraSoundTaskTCB;
StaticTask_t xGearMotorTaskTCB;
StaticTask_t xBTreceiveTCB;
StaticTask_t xARMotorTCB;
float distance1 = 0;
float distance2 = 0;

/**
 * @todo Show your control outcome of the M3508 motor as follows
 */
void motorTask(void *)
{
    // TODO: motorset.setCurrentLimit();
    int delay = 1;
    while (true)
    {
        TRControl::wholeTRControl(delay);
        vTaskDelay(delay);  // Delay and block the task for 1ms.
    }
}

void ultraSoundTask(void *)
{
    //HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
    HCSR04::hcsr04_init();
    while (true)
    {
        MG996R::setServoAngle(180);
        HCSR04::SendSingnal_1();
        vTaskDelay(50);
        distance1 = HCSR04::HCSR04_Read();
        HCSR04::SendSingnal_2();
        vTaskDelay(50);
        distance2 = HCSR04::HCSR04_Read();
        vTaskDelay(10);  // Delay and block the task for 1ms.
    }
}

void gearMotorTask(void *)
{
    //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    // HAL_TIM_Base_Start(&htim3);
    //HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
    // MG996R::setServoAngle(0);
    int angle = 0;
    while (true)
    {
        MG996R::setServoAngle(angle);
        angle = (angle + 20) % 180;
        // distance = HCSR04::HCSR04_Read();
        vTaskDelay(100);  // Delay and block the task for 100ms.
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
{ MG996R::setServoAngle(180);
    while (true)
    {
        
        
        ARControl::run();
        vTaskDelay(1);
    }
}
void ARMotorTask(void *)
{
    while (true)
    {
       
        
        ARMotor::MotorTask();
        vTaskDelay(1);
    }
}

void BTreceive(void *)
{
    while (true)
    {
        // For test use
        
        HC05::init();
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

    //HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
     
    DJIMotor::init();  // Initalize the DJIMotor driver
    DR16::init();      // Intialize the DR16 driver
    
    // xTaskCreateStatic(
    //     motorTask, "motorTask", 256, NULL, 1, uxPIDTaskStack, &xPIDTaskTCB);
    xTaskCreateStatic(CANReceiveTask,
                      "CANReceiveTask",
                      256,
                      NULL,
                      1,
                      uxReceiveTaskStack,
                      &xReceiveTaskTCB);
    xTaskCreateStatic(
        ARTask, "ARTask", 512, NULL, 1, uxARTaskStack, &xARTaskTCB);
        xTaskCreateStatic(
        ARMotorTask, "ARMotorTask", 512, NULL, 1, uxARMotorStack, &xARMotorTCB);
    // xTaskCreateStatic(
    //     ultraSoundTask,
    //     "ultraSoundTask",
    //     256,
    //     NULL,
    //     1,
    //     uxUltraSoundTaskStack,
    //     &xUltraSoundTaskTCB);  // Add the main task into the scheduler
xTaskCreateStatic(
         BTreceive, "BTreceive", 256, NULL, 1, uxBTreceiveStack, &xBTreceiveTCB);
        //  xTaskCreateStatic(
        //  gearMotorTask, "gearMotorTask", 256, NULL, 1, uxGearMotorTaskStack, &xGearMotorTaskTCB);
    /**
     * @todo Add your own task here
     */
}
