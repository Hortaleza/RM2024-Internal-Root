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

int RunRPM = 5000;
int Lstatus = 0;
int Rstatus = 0;
int status = 1;
int status2 = 1;
int servoCount = 1;
int allowToProceed = 0;
int reachBlackLine = 0;
uint32_t startTime = 0;
bool received = false;
// uint8_t box1;
// uint8_t box2;
//uint32_t stopTime = 1000;
static Control::PID motorPID[2] = {
    {Kp, Ki, Kd}, {Kp, Ki, Kd}
};
// id of left motor is 1, right is 2
void updateStatus()
{

     Lstatus = HAL_GPIO_ReadPin(GPIOA, LPIN);
     Rstatus = HAL_GPIO_ReadPin(GPIOA, RPIN);
    //box1 = HC05::boxesChosen[0];
    //box2 = HC05::boxesChosen[1];
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

void turnLeft(float degree){
    // if (status){
    //     startTime = HAL_GetTick(); 
    //     status = 0;
    // }
    //     if ((HAL_GetTick() - startTime) < runTime) {
            
    //         right(3000);
    //         DJIMotor::motorset.transmit();
    //     } else if ((HAL_GetTick() - startTime) < (runTime + stopTime)) {
            
    //         stop();
    //         DJIMotor::motorset.transmit();
    //     } else {
            
    //         startTime = HAL_GetTick();
    //     }

        
    

}

//bool stopped = 0;
void aimBox(float turn, float forward, int direction){
    if (direction == 1){
    ARMotor::left(1000);
    }
    if (direction == -1){
    ARMotor::right(1000);
    }
        vTaskDelay(turn);
        ARMotor::stop();
        vTaskDelay(100);
        ARMotor::forward(3000);
        vTaskDelay(forward);
        ARMotor::stop();
        vTaskDelay(200);
        if (servoCount == 1){
            for (int i =180; i>=90;i--){
                MG996R::setServoAngle(i);
                vTaskDelay(10);
            }
        
        servoCount++;
        }
        else{
            for (int i =90; i>=0;i--){
                MG996R::setServoAngle(i);
                vTaskDelay(10);
            }
        }
        vTaskDelay(1000);
        ARMotor::forward(-3000);
        vTaskDelay(forward);
        if (direction == 1){
        ARMotor::right(1000);
        }
        if (direction == -1){
            ARMotor::left(1000);
        }
        vTaskDelay(turn);
        ARMotor::stop();
        vTaskDelay(500);
}
void run(){
    updateStatus();
    if ((reachBlackLine==0 && allowToProceed) || (reachBlackLine==1 && allowToProceed)){
    if ( Lstatus && Rstatus ) {
        // if (status){
        // startTime = HAL_GetTick(); 
        //  status = 0;
        // }
        // uint32_t stopTime = startTime+100;
        // if(!stopped && HAL_GetTick()<stopTime){
        //     stop();
        //     DJIMotor::motorset.transmit();
        // }
        
        // else if (HAL_GetTick()>stopTime) {
        //     stopped = 1;
        //     reachBlackLine = 2;
        // }
        if (reachBlackLine == 0){
            ARMotor::forward(3000);
            vTaskDelay(350);
            reachBlackLine++;
        }
        else if (reachBlackLine == 1){
            ARMotor::stop();
            reachBlackLine = 2;
        }
        
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
    //27.6 9.74
    else if (reachBlackLine==2 && allowToProceed){
        for (int i=0; i<2; i++){
        if (HC05::boxesChosen[i] =='4'){
        aimBox(420, 1350,1);
        }
        else if (HC05::boxesChosen[i] =='3'){
        aimBox(90, 1250,1);
        }
        else if (HC05::boxesChosen[i] =='2'){
        aimBox(195, 1250,-1);
        }
        else if (HC05::boxesChosen[i] =='1'){
        aimBox(600, 1350,-1);
        }
        }
        allowToProceed=0;



        // for (int i=0; i<2; i++){
        //     if (0){
        //         status = 1;
        //         if (status){
        // startTime = HAL_GetTick(); 
        //  status = 0;
        // }
        // uint32_t stopTime = startTime+100;
        // if(!stopped && HAL_GetTick()<stopTime){
        //     stop();
        //     DJIMotor::motorset.transmit();
        // }
        //         uint32_t stopTime = HAL_GetTick()+214.66667;
        //         while (HAL_GetTick()<stopTime){
        //         left(3000);
        //         DJIMotor::motorset.transmit();
        //         }
        //         while (HAL_GetTick()<stopTime+100){
        //         stop(); 
        //         DJIMotor::motorset.transmit();
        //         }
        //         while (HAL_GetTick()<stopTime+200){
        //         forward(3000);
        //         DJIMotor::motorset.transmit();
        //         }
        //         while (HAL_GetTick()<stopTime+300){
        //         stop(); 
        //         DJIMotor::motorset.transmit();
        //         }
        //         while (HAL_GetTick()<stopTime+400){
        //         forward(-3000);
        //         DJIMotor::motorset.transmit();
        //         }
        //         while (HAL_GetTick()<stopTime+500){
        //         stop(); 
        //         DJIMotor::motorset.transmit();
        //         }
        //         while (HAL_GetTick()<stopTime+714.66667){
        //         right(3000);
        //         DJIMotor::motorset.transmit();
        //         }
        //         while (HAL_GetTick()<stopTime+814.6667){
        //         stop(); 
        //         DJIMotor::motorset.transmit();
        //         }
                


        //     }
        //     else if (0){
                
        //         //turnLeft(9.74);
        //     }
        // }
        // allowToProceed = 0;

    }
    if (HC05::boxesChosen[0] =='1' || HC05::boxesChosen[0] =='2' || HC05::boxesChosen[0] =='3' || HC05::boxesChosen[0] =='4'){
        if (!received){
        allowToProceed = 1;
        received = true;
        }

    }
}
void turnFixedDistance(uint32_t runTime, int speed){
    // if (status2){
    //     startTime = HAL_GetTick(); 
    //     status2 = 0;
    // }
    //     if ((HAL_GetTick() - startTime) < runTime) {
            
    //         forward(speed);
    //         DJIMotor::motorset.transmit();
    //     } else if ((HAL_GetTick() - startTime) < (runTime + stopTime)) {
            
    //         stop();
    //         DJIMotor::motorset.transmit();
    //     } else {
            
    //         startTime = HAL_GetTick();
    //     }

}
void turn(uint32_t runTime){
    // if (status){
    //     startTime = HAL_GetTick(); 
    //     status = 0;
    // }
    //     if ((HAL_GetTick() - startTime) < runTime) {
            
    //         right(3000);
    //         DJIMotor::motorset.transmit();
    //     } else if ((HAL_GetTick() - startTime) < (runTime + stopTime)) {
            
    //         stop();
    //         DJIMotor::motorset.transmit();
    //     } else {
            
    //         startTime = HAL_GetTick();
    //     }

        
    
}
}

#endif