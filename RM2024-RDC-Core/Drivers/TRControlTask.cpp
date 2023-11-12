#include "TRControlTask.hpp"

#if IS_TR

namespace TRControl
{
// Get mode from DR16 and choose based on modes

int16_t currentRPM[8] = {};
int16_t targetRPM[8]  = {};
int indices[4]        = {FR, FL, BL, BR};

int previousMode = 2;

// PID
float Kp                        = 10;
float Ki                        = 2;
float Kd                        = 0.02;

static Control::PID motorPID[4] = {
    {Kp, Ki, Kd}, {Kp, Ki, Kd}, {Kp, Ki, Kd}, {Kp, Ki, Kd}
};

static Control::PID accuratePID[4] = {
    {Kp, Ki, Kd}, {Kp, Ki, Kd}, {Kp, Ki, Kd}, {Kp, Ki, Kd} // Needs to modify
};



void WholeTRControl(int delay)
{
    // Check mode
    int modeNow = DR16::uniformed.s2;
    if (modeNow != previousMode)
    {
        // re-initialize all PID modules
        // How to avoid shaking when switching modes?
        for (int i = 0; i < 4; i++)
        {
            motorPID[i].clear();
            accuratePID[i].clear();
        }
        // switch mode

        previousMode = modeNow;
    }

    // Goto the mode
    if (previousMode == 2)
    {
        runFastMode(delay);
        return;
    }
    if (previousMode == 1)
    {
        runAccurateMode(delay);
        return;
    }
}

void runFastMode(int delay)
{
    // Check if connected

    bool connected = DR16::getConnectionStatus(100);

    if (!connected)
    {
        for (int i = 0; i < 8; i++) {
            targetRPM[i] = 0;
        }
        return;
    }

    /* ===== Wheels control ===== */

    // Calculate RPM using Mecanum wheels algorithm

    double forward = DR16::uniformed.channel1;
    double strafe  = DR16::uniformed.channel0;
    double turn    = DR16::uniformed.channel2;

    double targetRatio[4];
    
    // @todo: Needs to reset the signs to match our motor setup!!

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

void runAccurateMode(int delay)
{
    
}

}  // namespace TRControl

#endif