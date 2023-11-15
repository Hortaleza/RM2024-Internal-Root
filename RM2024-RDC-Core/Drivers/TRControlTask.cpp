#include "TRControlTask.hpp"

#if IS_TR

namespace TRControl
{
// Get mode from DR16 and choose based on modes

int16_t currentRPM[8] = {};
float targetRPM[8]  = {};

double targetRatio[8] = {};

float stable[2] = {};

const int indices[4]  = {FR, FL, BL, BR};

int left_mode  = 2;
int right_mode = 2;

// PID

static Control::PID motorPID[4] = {
    {Kp, Ki, Kd}, {Kp, Ki, Kd}, {Kp, Ki, Kd}, {Kp, Ki, Kd}
};


static Control::PID armPID[2] = {
    {Kp_arm, Ki_arm, Kd_arm}, {Kp_arm, Ki_arm, Kd_arm}
};

// Auto mode

static Control::PID autoPID = {Kp_aut, Ki_aut, Kd_aut};

float target_distance;
// To be determined when mode is switched

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
        // Do not clear motorPID!
        
        // Clear armPID
        armPID[0].clear();
        armPID[1].clear();

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

bool stopping[2] = {0, 0};
bool previousStatus[2] = {0, 0};
inline uint16_t abs(uint16_t a) { return (a > 0) ? a : -a; }

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

    // @todo: Needs to reset the signs to match our motor setup!!

    targetRatio[indices[0]] = -(forward - strafe - turn);
    targetRatio[indices[1]] = (forward + strafe + turn);
    targetRatio[indices[2]] = (forward - strafe + turn);
    targetRatio[indices[3]] = -(forward + strafe - turn);

    // Find the one with the maximum absolute value among the four
    double maxabs = (abs(forward) + abs(strafe) + abs(turn));
    if (maxabs >= 1)
    {
        for (int i = 0; i < 4; i++)
        {
            targetRatio[indices[i]] /= maxabs;
        }
    }

    // Set RPM
    for (int _ = 0; _ < 4; _++)
    {
        int i         = indices[_];
        targetRPM[i]  = targetRatio[i] * DJIMotor::MAX_RPM * speed;
        currentRPM[i] = DJIMotor::motorset[i].getRPM();
        DJIMotor::motorset[i].setCurrent(
            motorPID[_].update(targetRPM[i], currentRPM[i], delay));
    }

    // Arm control


    targetRPM[ARM_VERTICAL]   = 0;
    targetRPM[ARM_HORIZONTAL] = 0;

    bool newStatus[2] = {0, 0};
    const float controlLimit = 0.5;

    if (DR16::uniformed.channel3 > controlLimit) {
        targetRPM[ARM_VERTICAL] = -DJIMotor::MAX_RPM * ARMSpeed;
        newStatus[0]               = 1;
    }
    else if (DR16::uniformed.channel3 < -controlLimit) {
        targetRPM[ARM_VERTICAL] = DJIMotor::MAX_RPM * ARMSpeed;
        newStatus[0]               = 1;
    }

    if (DR16::uniformed.channel2 > controlLimit) {
        targetRPM[ARM_HORIZONTAL] = DJIMotor::MAX_RPM * ARMSpeed;
        newStatus[1]                 = 1;
    }
    else if (DR16::uniformed.channel2 < -controlLimit) {
        targetRPM[ARM_HORIZONTAL] = -DJIMotor::MAX_RPM * ARMSpeed;
        newStatus[1]                 = 1;
    }
    
    if (previousStatus[0] != newStatus[0] && newStatus[0] == 0)
    {
        stopping[0] = 0;
    }
    if (previousStatus[1] != newStatus[1] && newStatus[1] == 0)
    {
        stopping[1] = 0;
    }

    previousStatus[0] = newStatus[0];
    previousStatus[1] = newStatus[1];

    currentRPM[ARM_VERTICAL] = DJIMotor::motorset[ARM_VERTICAL].getRPM();
    currentRPM[ARM_HORIZONTAL] = DJIMotor::motorset[ARM_HORIZONTAL].getRPM();

    float o1 = armPID[0].getAttemptedUpdate(
        targetRPM[ARM_VERTICAL], currentRPM[ARM_VERTICAL], delay);
    float o2 = armPID[1].getAttemptedUpdate(
        targetRPM[ARM_HORIZONTAL], currentRPM[ARM_HORIZONTAL], delay);

    // @todo: Check if stable and acquire stable output to replace the output limit

    const int16_t OUTPUT_LIMIT1 = 7000;
    const int16_t OUTPUT_LIMIT2 = 7000;
    const float speedLimit      = 0.95;

    if ((abs(currentRPM[ARM_VERTICAL]) >
         speedLimit * DJIMotor::MAX_RPM * ARMSpeed) &&
        abs(o1) > OUTPUT_LIMIT1)
    {
        stopping[0] = 1;
    }
    if ((abs(currentRPM[ARM_HORIZONTAL]) >
         speedLimit * DJIMotor::MAX_RPM * ARMSpeed) &&
        abs(o2) > OUTPUT_LIMIT2)
    {
        stopping[1] = 1;
    }

    if (stopping[0] == 1) {
        targetRPM[ARM_VERTICAL] = 0;
    }
    if (stopping[1] == 1) {
        targetRPM[ARM_HORIZONTAL] = 0;
    }

    DJIMotor::motorset[ARM_VERTICAL].setCurrent(armPID[0].update(
        targetRPM[ARM_VERTICAL], currentRPM[ARM_VERTICAL], delay));
    DJIMotor::motorset[ARM_HORIZONTAL].setCurrent(armPID[1].update(
        targetRPM[ARM_HORIZONTAL], currentRPM[ARM_HORIZONTAL], delay));

    DJIMotor::motorset
        .transmit();  // Transmit the data to the motor in a package
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