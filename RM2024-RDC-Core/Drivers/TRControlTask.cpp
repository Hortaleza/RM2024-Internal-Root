#include "TRControlTask.hpp"

#if IS_TR

namespace TRControl
{
// Get mode from DR16 and choose based on modes

int16_t currentRPM[8] = {};
int16_t targetRPM[8]  = {};

void runFastMode(int delay)
{
    // Wheels control
    float Kp                = 10;
    float Ki                = 2;
    float Kd                = 0.02;
    static Control::PID motorPID[4] = {
        {Kp, Ki, Kd}, {Kp, Ki, Kd}, {Kp, Ki, Kd}, {Kp, Ki, Kd}
    };

    int indices[4] = {FR, FL, BL, BR};

    // Check if connected

    bool connected = DR16::getConnectionStatus(100);

    if (!connected)
    {
        for (int i = 0; i < 8; i++) {
            targetRPM[i] = 0;
        }
        return;
    }

    // Calculate RPM using Mecanum wheels algorithm

    double forward = DR16::uniformed.channel1;
    double strafe  = DR16::uniformed.channel0;
    double turn    = DR16::uniformed.channel2;

    double targetRatio[4];

    targetRatio[indices[0]] = (forward - strafe - turn) / 3.0;
    targetRatio[indices[1]] = (forward + strafe + turn) / 3.0;
    targetRatio[indices[2]] = (forward - strafe + turn) / 3.0;
    targetRatio[indices[3]] = (forward + strafe - turn) / 3.0;

    // Find the one with the maximum absolute value among the four
    double maxabs = (abs(forward) + abs(strafe) + abs(turn)) / 3.0;
    if (3*maxabs >= 1) {
        targetRatio[indices[0]] /= maxabs;
        targetRatio[indices[1]] /= maxabs;
        targetRatio[indices[2]] /= maxabs;
        targetRatio[indices[3]] /= maxabs;
    }

    // Set RPM

    for (int _ = 0; _ < 4; _++)
    {
        int i                 = indices[_];
        targetRPM[i]          = targetRatio[i] * DJIMotor::MAX_RPM;
        currentRPM[i]         = DJIMotor::motorset[i].getRPM();
        DJIMotor::motorset[i].setCurrent(
            motorPID[_].update(targetRPM[i], currentRPM[i], delay/1000.0f));
    }


    // Arm control






    DJIMotor::motorset.transmit();  // Transmit the data to the motor in a package
}

}  // namespace TRControl

#endif