/**
 * @file PID.hpp
 * @author - GUO, Zilin
 *         - JIANG, Yicheng
 *         - Your name
 * @date 2023-10-20
 * @brief This file gives out the template skeleton code for you to implement a
 * PID algorithm
 */
#pragma once
#include "AppConfig.h"
#include "can.h"
float abs(float n);
double abs(double n);
#if USE_PID
#include "FreeRTOS.h"
#include "task.h"

    namespace Control
{

class PID
{
   public:
    /**
     * @brief Constructor function
     * @param Kp_ The P term of the PID
     * @param Ki_ The I term of the PID
     * @param Kd_ The D term of the PID
     */
    PID(float Kp_ = 0, float Ki_ = 0, float Kd_ = 0)
        : Kp(Kp_), Ki(Ki_), Kd(Kd_){};
    /**
     * @brief Update the PID status and return
     * @note  You need to implement the specific definition in the PID.cpp file
     * @param target        The target value
     * @param measurement   The feedback value
     * @param dt            The time interval between two updates
     */
    float update(float target, float measurement, float dt = 1);
    float getAttemptedUpdate(float target, float measurement, float dt);

    float getOutput() { return output; }
    /*===================*/
    // Your self-defined functions begin here
    // You might wish to add more functions for your PID module
    /*===================*/

    void clear();

    /*===================*/
    // Your self-defined functions end here
    /*===================*/

   private:
    float Kp = 0;  // The P param of the PID
    float Ki = 0;  // The I param of the PID
    float Kd = 0;  // The D param of the PID

    float error     = 0;  // The error in this update
    float lastError = 0;  // The error from last update

    float pOut = 0;  // The P term output of the PID
    float iOut = 0;  // The I term output of the PID
    float dOut = 0;  // The D term output of the PID
    float integral = 0;

    float output = 0;  // The current output of the PID

    float maxoutput = 16384;
    float minoutput = -16384;

    float a = 0.9;
    float previousfilter = 0;
    float currentfilter = 0;

    /*===================*/
    // Your self-defined variables begin here
    /*===================*/

    /*===================*/
    // Your self-defined variables end here
    /*===================*/
};
}  // namespace Control

#endif
