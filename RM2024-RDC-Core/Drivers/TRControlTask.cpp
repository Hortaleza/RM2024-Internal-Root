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
float Kp = 12;
float Ki = 0.6;
float Kd = 10;

static Control::PID motorPID[4] = {
    {Kp, Ki, Kd}, {Kp, Ki, Kd}, {Kp, Ki, Kd}, {Kp, Ki, Kd}
};

float Kp_arm = 12;
float Ki_arm = 0.6;
float Kd_arm = 10;

static Control::PID armPID = {Kp_arm, Ki_arm, Kd_arm};

void WholeTRControl(int delay)
{
    // Check mode
    int modeNow = DR16::uniformed.s2;

    // If switched
    if (modeNow != previousMode)
    {
        /* ===== re-initialize all PID modules ===== */
        // How to avoid shaking when switching modes?
        for (int i = 0; i < 4; i++)
        {
            motorPID[i].clear();
        }
        armPID.clear();

        // update mode status

        previousMode = modeNow;
    }

    // Goto the mode
    if (previousMode == 2)
    {
        runNormalMode(1, delay);
        return;
    }
    if (previousMode == 1)
    {
        // Slow moving
        runNormalMode(0.1, delay);
        return;
    }
    if (previousMode == 3)
    {
        // Slow moving plus arm control (without turning)
        runArmMode(0.1, delay);
        return;
    }
}

inline double signedSquare(double a) { return (a > 0) ? a * a : -a * a; }

void runNormalMode(float speed, int delay)
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

    double forward = signedSquare(DR16::uniformed.channel1);
    double strafe  = signedSquare(DR16::uniformed.channel0);
    double turn    = signedSquare(DR16::uniformed.channel2);

    double targetRatio[4];
    
    // @todo: Needs to reset the signs to match our motor setup!!

    targetRatio[0] = -(forward - strafe - turn);
    targetRatio[1] = (forward + strafe + turn);
    targetRatio[2] = (forward - strafe + turn);
    targetRatio[3] = -(forward + strafe - turn);

    // Find the one with the maximum absolute value among the four
    double maxabs = (abs(forward) + abs(strafe) + abs(turn));
    if (maxabs >= 1) {
        targetRatio[0] /= maxabs;
        targetRatio[1] /= maxabs;
        targetRatio[2] /= maxabs;
        targetRatio[3] /= maxabs;
    }

    // Set RPM
    for (int _ = 0; _ < 4; _++)
    {
        int i                 = indices[_];
        targetRPM[i]          = targetRatio[i] * DJIMotor::MAX_RPM * speed;
        currentRPM[i]         = DJIMotor::motorset[i].getRPM();
        DJIMotor::motorset[i].setCurrent(
            motorPID[_].update(targetRPM[i], currentRPM[i], delay));
    }


    DJIMotor::motorset.transmit();  // Transmit the data to the motor in a package
}

void runArmMode(float speed, int delay)
{
    // Check if connected

    bool connected = DR16::getConnectionStatus(100);

    if (!connected)
    {
        for (int i = 0; i < 8; i++)
        {
            targetRPM[i] = 0;
        }
        return;
    }

    /* ===== Wheels control ===== */

    // Calculate RPM using Mecanum wheels algorithm

    double forward = signedSquare(DR16::uniformed.channel1);
    double strafe  = signedSquare(DR16::uniformed.channel0);
    double turn    = 0;

    double targetRatio[4];

    // @todo: Needs to reset the signs to match our motor setup!!

    targetRatio[0] = -(forward - strafe - turn);
    targetRatio[1] = (forward + strafe + turn);
    targetRatio[2] = (forward - strafe + turn);
    targetRatio[3] = -(forward + strafe - turn);

    // Find the one with the maximum absolute value among the four
    double maxabs = (abs(forward) + abs(strafe) + abs(turn));
    if (maxabs >= 1)
    {
        targetRatio[0] /= maxabs;
        targetRatio[1] /= maxabs;
        targetRatio[2] /= maxabs;
        targetRatio[3] /= maxabs;
    }

    // Set RPM
    for (int _ = 0; _ < 4; _++)
    {
        int i         = indices[_];
        targetRPM[i]  = targetRatio[_] * DJIMotor::MAX_RPM * speed;
        currentRPM[i] = DJIMotor::motorset[i].getRPM();
        DJIMotor::motorset[i].setCurrent(
            motorPID[_].update(targetRPM[i], currentRPM[i], delay));
    }

    // Arm control

    double targetRatioARM = DR16::uniformed.channel3;
    float speedARM = 0.002;

    targetRPM[ARM] = targetRatioARM * DJIMotor::MAX_RPM * speedARM;
    currentRPM[ARM] = DJIMotor::motorset[ARM].getRPM();
    DJIMotor::motorset[ARM].setCurrent(
        motorPID[ARM].update(targetRPM[ARM], currentRPM[ARM], delay));



    DJIMotor::motorset.transmit();  // Transmit the data to the motor in a package
}

void runAutoMode()
{
    // must be completely auto control
    

}

}  // namespace TRControl

#endif