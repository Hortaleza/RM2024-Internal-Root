#include "ARControlTask.hpp"

#if IS_AR
namespace ARControl
{
    
uint16_t RPIN = GPIO_PIN_6;
uint16_t LPIN = GPIO_PIN_7;
//static volatile bool status = 0;


static volatile int rpmTest;
static volatile int positionTest;

float Kp                        = 5;
float Ki                        = 0;
float Kd                        = 0;

int RunRPM = 3000;
int Lstatus = 0;
int Rstatus = 0;
int status = 1;
int status2 = 1;
uint32_t startTime = 0;

uint32_t stopTime = 1000;
static Control::PID motorPID[2] = {
    {Kp, Ki, Kd}, {Kp, Ki, Kd}
};
// id of left motor is 1, right is 2
void updateStatus()
{

    Lstatus = HAL_GPIO_ReadPin(GPIOA, LPIN);
    Rstatus = HAL_GPIO_ReadPin(GPIOA, RPIN);
   
    // Black: status = 0; White: status = 1;
}

void forward(int RPM){
    int targetRPM0  = RPM;
    int targetRPM1  = -RPM;
    int currentRPML = DJIMotor::motorset[0].getRPM();
    DJIMotor::motorset[0].setCurrent(
        motorPID[0].update(targetRPM0, currentRPML, 1));
    int currentRPMR = DJIMotor::motorset[1].getRPM();
    DJIMotor::motorset[1].setCurrent(
        motorPID[1].update(targetRPM1, currentRPMR, 1));
}

void left(int RPM){
    int targetRPM = RPM;
    int currentRPML = DJIMotor::motorset[0].getRPM();
        DJIMotor::motorset[0].setCurrent(
            motorPID[0].update(-targetRPM, currentRPML, 1));
    int currentRPMR = DJIMotor::motorset[1].getRPM();
        DJIMotor::motorset[1].setCurrent(
            motorPID[1].update(-targetRPM, currentRPMR, 1));
    
}

void right(int RPM){
    int targetRPM = RPM;
    int currentRPML = DJIMotor::motorset[0].getRPM();
        DJIMotor::motorset[0].setCurrent(
            motorPID[0].update(targetRPM, currentRPML, 1));
    int currentRPMR = DJIMotor::motorset[1].getRPM();
        DJIMotor::motorset[1].setCurrent(
            motorPID[1].update(targetRPM, currentRPMR, 1));
    
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

void test(){
    positionTest = DJIMotor::motorset[0].getPosition();

}

void run(){
    updateStatus();
    if ( Lstatus && Rstatus ) {
        stop();
        DJIMotor::motorset.transmit();
      }
    
    if ( Lstatus && !Rstatus ) {
        left(RunRPM);
        DJIMotor::motorset.transmit();
      }
    
    if ( !Lstatus && Rstatus ) {
        right(RunRPM);
        DJIMotor::motorset.transmit();
      }
    
    if ( !Lstatus && !Rstatus ) {
        forward(RunRPM);
        DJIMotor::motorset.transmit();
      }
}
void turnFixedDistance(uint32_t runTime, int speed){
    if (status2){
        startTime = HAL_GetTick(); 
        status2 = 0;
    }
        if ((HAL_GetTick() - startTime) < runTime) {
            
            forward(speed);
            DJIMotor::motorset.transmit();
        } else if ((HAL_GetTick() - startTime) < (runTime + stopTime)) {
            
            stop();
            DJIMotor::motorset.transmit();
        } else {
            
            startTime = HAL_GetTick();
        }

}
void turn(uint32_t runTime){
    if (status){
        startTime = HAL_GetTick(); 
        status = 0;
    }
        if ((HAL_GetTick() - startTime) < runTime) {
            
            right(3000);
            DJIMotor::motorset.transmit();
        } else if ((HAL_GetTick() - startTime) < (runTime + stopTime)) {
            
            stop();
            DJIMotor::motorset.transmit();
        } else {
            
            startTime = HAL_GetTick();
        }

        
    
}
}

#endif