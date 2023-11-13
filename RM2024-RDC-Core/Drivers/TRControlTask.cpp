#include "TRControlTask.hpp"

#if IS_TR

namespace TRControl
{
// Get mode from DR16 and choose based on modes

int16_t currentRPM[8] = {};
int16_t targetRPM[8]  = {};
const int indices[4]  = {FR, FL, BL, BR};

int left_mode  = 2;
int right_mode = 2;

// PID
const float Kp = 12;
const float Ki = 0;
const float Kd = 45;

static Control::PID motorPID[4] = {
    {Kp, Ki, Kd}, {Kp, Ki, Kd}, {Kp, Ki, Kd}, {Kp, Ki, Kd}
};

const float Kp_arm = 10;
const float Ki_arm = 0;
const float Kd_arm = 0;

static Control::PID armPID = {Kp_arm, Ki_arm, Kd_arm};


// Auto mode

const float Kp_aut = 10;
const float Ki_aut = 0;
const float Kd_aut = 0;

static Control::PID autoPID = {Kp_aut, Ki_aut, Kd_aut};

float target_distance = 12; // To be determined when mode is switched 






void wholeTRControl(int delay)
{
    // Check if connected

    bool connected = DR16::getConnectionStatus(100);

    if (!connected)
    {
        for (int i = 0; i < 8; i++)
        {
            targetRPM[i] = 0;
        }
        // Simply set all current to zero instead of using PID to control to zero
        for (int i = 0; i < 8; i++)
        {
            currentRPM[i] = DJIMotor::motorset[i].getRPM();
            DJIMotor::motorset[i].setCurrent(0);
        }
        DJIMotor::motorset.transmit();
        return;
    }

    // Check mode
    int new_left_mode  = DR16::uniformed.s1;
    int new_right_mode = DR16::uniformed.s2;

    /* ===== If switched, re-initialize all PID modules ===== */
    if (new_right_mode != right_mode || new_left_mode != left_mode)
    {
        // Clear motorPID
        for (int i = 0; i < 4; i++)
        {
            motorPID[i].clear();
        }

        // Clear armPID
        armPID.clear();

        // Clear autoPID
        autoPID.clear();

        // update mode status
        left_mode  = new_left_mode;
        right_mode = new_right_mode;
    }

    // @todo: How to avoid shaking when switching modes?

    /* ===== Goto the mode ===== */

    // Check if is auto mode
    if (left_mode == 3)
    {
        runAutoMode(delay);
        return; // This return is a MUST!!!
    }

    // If manual then go to the specific one
    if (right_mode == 2)
    {
        runNormalMode(1, delay);
        return;
    }
    if (right_mode == 1)
    {
        // Slow moving
        runNormalMode(0.1, delay);
        return;
    }
    if (right_mode == 3)
    {
        // Slow moving plus arm control (without turning)
        runArmMode(0.1, delay);
        return;
    }
}

inline double signedSquare(double a) { return (a > 0) ? a * a : -a * a; }

void runNormalMode(float speed, int delay)
{

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

void runAutoMode(int delay)
{
    // must be completely auto control

    // @todo: If arriving at the destination, stop
    if (0)
    {

    }

    int16_t forward = 200;

    float measured_distance; // @todo: Use ultrasound

    int16_t turn = autoPID.update(target_distance, measured_distance, delay);

    // @todo: Needs to reset the signs to match our motor setup!

    targetRPM[indices[0]] = -(forward - turn);
    targetRPM[indices[1]] = (forward + turn);
    targetRPM[indices[2]] = (forward + turn);
    targetRPM[indices[3]] = -(forward - turn);

    for (int _ = 0; _ < 4; _++)
    {
        int i         = indices[_];
        currentRPM[i] = DJIMotor::motorset[i].getRPM();
        DJIMotor::motorset[i].setCurrent(
            motorPID[_].update(targetRPM[i], currentRPM[i], delay));
    }

    DJIMotor::motorset.transmit();  // Transmit the data to the motor in a package
}

}  // namespace TRControl

#endif