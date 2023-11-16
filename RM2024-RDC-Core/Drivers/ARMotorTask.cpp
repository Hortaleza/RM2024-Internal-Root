#include "ARMotorTask.hpp"

#if IS_AR
namespace ARMotor
{
    float Kp                        = 5;
float Ki                        = 0;
float Kd                        = 0;

static Control::PID motorPID[2] = {
    {Kp, Ki, Kd}, {Kp, Ki, Kd}
};

bool forwardStatus = false;
    bool leftStatus = false;
    bool rightStatus = false;
    bool stopStatus = false;
    int forwardSpeed = 0;
    int leftSpeed = 0;
    int rightSpeed = 0; 
    void forward(int RPM){
        forwardStatus = true;
        leftStatus = false;
        rightStatus = false;
        stopStatus = false;
        forwardSpeed = RPM;
    }
    void left(int RPM){
        leftStatus = true;
        forwardStatus = false;
        rightStatus = false;
        stopStatus = false;
        leftSpeed = RPM;
    }
    void right(int RPM){
        rightStatus = true;
        forwardStatus = false;
        leftStatus = false;     
        stopStatus = false;
        rightSpeed = RPM;
    }
    void stop(){
        stopStatus = true;
        leftStatus = false;
        forwardStatus = false;
        rightStatus = false;
    }
    
    void MotorTask(){
        if (forwardStatus){
        int targetRPM0  = forwardSpeed;
    int targetRPM1  = -forwardSpeed;
    int currentRPML = DJIMotor::motorset[0].getRPM();
    DJIMotor::motorset[0].setCurrent(
        motorPID[0].update(targetRPM0, currentRPML, 1));
    int currentRPMR = DJIMotor::motorset[1].getRPM();
    DJIMotor::motorset[1].setCurrent(
        motorPID[1].update(targetRPM1, currentRPMR, 1));
        DJIMotor::motorset.transmit();
        }
        if (leftStatus){
            int targetRPM = leftSpeed;
    int currentRPML = DJIMotor::motorset[0].getRPM();
        DJIMotor::motorset[0].setCurrent(
            motorPID[0].update(-targetRPM, currentRPML, 1));
    int currentRPMR = DJIMotor::motorset[1].getRPM();
        DJIMotor::motorset[1].setCurrent(
            motorPID[1].update(-targetRPM, currentRPMR, 1));
            DJIMotor::motorset.transmit();
        }
        if (rightStatus){
            int targetRPM = rightSpeed;
    int currentRPML = DJIMotor::motorset[0].getRPM();
        DJIMotor::motorset[0].setCurrent(
            motorPID[0].update(targetRPM, currentRPML, 1));
    int currentRPMR = DJIMotor::motorset[1].getRPM();
        DJIMotor::motorset[1].setCurrent(
            motorPID[1].update(targetRPM, currentRPMR, 1));
            DJIMotor::motorset.transmit();
        }
        if (stopStatus){
             int targetRPM = 0;
    int currentRPML = DJIMotor::motorset[0].getRPM();
        DJIMotor::motorset[0].setCurrent(
            motorPID[0].update(targetRPM, currentRPML, 1));
    int currentRPMR = DJIMotor::motorset[1].getRPM();
        DJIMotor::motorset[1].setCurrent(
            motorPID[1].update(-targetRPM, currentRPMR, 1));
            DJIMotor::motorset.transmit();
        }


    }

}
#endif