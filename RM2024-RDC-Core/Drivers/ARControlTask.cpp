#include "ARControlTask.hpp"

#if IS_AR
namespace ARControl
{
    uint16_t A0PIN = GPIO_PIN_6;
uint16_t LPIN = GPIO_PIN_7;
uint16_t RPIN = GPIO_PIN_6;
static volatile bool status = 0;

float Kp                        = 12;
float Ki                        = 0.6;
float Kd                        = 10;

int RPM = 1000;
int Lstatus = 0;
int Rstatus = 0;
static Control::PID motorPID[2] = {
    {Kp, Ki, Kd}, {Kp, Ki, Kd}
};
// id of left motor is 0, right is 1
void updateStatus()
{
    Lstatus = HAL_GPIO_ReadPin(GPIOA, LPIN);
    Rstatus = HAL_GPIO_ReadPin(GPIOA, RPIN);
    // Black: status = 0; White status = 1;
}

void forward(){
    for (int i = 0; i < 2; i++)
    {
        
        int targetRPM = RPM;
        int currentRPM = DJIMotor::motorset[i].getRPM();
        DJIMotor::motorset[i].setCurrent(
            motorPID[i].update(targetRPM, currentRPM, 1));
    }
}

void left(){
    int targetRPM = RPM;
    int currentRPML = DJIMotor::motorset[0].getRPM();
        DJIMotor::motorset[0].setCurrent(
            motorPID[0].update(-targetRPM, currentRPML, 1));
    int currentRPMR = DJIMotor::motorset[1].getRPM();
        DJIMotor::motorset[1].setCurrent(
            motorPID[1].update(targetRPM, currentRPMR, 1));
    
}

void right(){
    int targetRPM = RPM;
    int currentRPML = DJIMotor::motorset[0].getRPM();
        DJIMotor::motorset[0].setCurrent(
            motorPID[0].update(targetRPM, currentRPML, 1));
    int currentRPMR = DJIMotor::motorset[1].getRPM();
        DJIMotor::motorset[1].setCurrent(
            motorPID[1].update(-targetRPM, currentRPMR, 1));
    
}

void stop(){
    int targetRPM = 0;
    int currentRPML = DJIMotor::motorset[0].getRPM();
        DJIMotor::motorset[0].setCurrent(
            motorPID[0].update(targetRPM, currentRPML, 1));
    int currentRPMR = DJIMotor::motorset[1].getRPM();
        DJIMotor::motorset[1].setCurrent(
            motorPID[1].update(-targetRPM, currentRPMR, 1));
}

void run(){
    updateStatus();
    if ( !Lstatus && !Rstatus ) {
        stop();
        DJIMotor::motorset.transmit();
      }
    
    if ( !Lstatus && Rstatus ) {
        left();
        DJIMotor::motorset.transmit();
      }
    
    if ( Lstatus && !Rstatus ) {
        right();
        DJIMotor::motorset.transmit();
      }
    
    if ( Lstatus && Rstatus ) {
        forward();
        DJIMotor::motorset.transmit();
      }
}
}
#endif