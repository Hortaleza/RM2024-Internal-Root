#include "HCSR04.hpp"
#if USE_HCSR04

namespace HCSR04
{
  #include "tim.h"
  #include "gpio.h"
  #include "stm32f1xx_hal.h"
  
  uint8_t IS_FIRST_CAPTURED = 0;
  uint32_t IC_Val1 = 0;
  uint32_t IC_Val2 = 0;
  uint32_t Difference = 0;
  uint16_t Distance = 0;

  void hcsr04_init()
  {
    HAL_TIM_RegisterCallback(&htim3, HAL_TIM_IC_CAPTURE_CB_ID, IC_CaptureCallback);
    HAL_TIM_RegisterCallback(&htim1, HAL_TIM_IC_CAPTURE_CB_ID, IC_CaptureCallback);
    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_Base_Start(&htim1);
    // HAL_TIM_Base_Start(&htim1);
    HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
    // HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 15);
  }

  void delay_us(uint16_t time,TIM_HandleTypeDef *htim)
  {
    __HAL_TIM_SET_COUNTER(htim,0);
    while(__HAL_TIM_GET_COUNTER(htim) < time);
    // while( time -- );
  }

  void SendSingnal_1()
  {
    HAL_GPIO_WritePin(TRIG_GPIO_Port,TRIG_Pin,GPIO_PIN_RESET);
    delay_us(2,&htim3);
    HAL_GPIO_WritePin(TRIG_GPIO_Port,TRIG_Pin,GPIO_PIN_SET);
    delay_us(20,&htim3); // 10us
    HAL_GPIO_WritePin(TRIG_GPIO_Port,TRIG_Pin,GPIO_PIN_RESET);
    __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_CC1);  
  }

  void SendSingnal_2()
  {
    HAL_GPIO_WritePin(TRIG2_GPIO_Port,TRIG2_Pin,GPIO_PIN_RESET);
    delay_us(2,&htim1);
    HAL_GPIO_WritePin(TRIG2_GPIO_Port,TRIG2_Pin,GPIO_PIN_SET);
    delay_us(20,&htim1); // 10us
    HAL_GPIO_WritePin(TRIG2_GPIO_Port,TRIG2_Pin,GPIO_PIN_RESET);
    __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_CC1);  
  }

  uint16_t HCSR04_Read(void)
  {
    return Distance;
  }

  void IC_CaptureCallback(TIM_HandleTypeDef *htim) 
  {
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) 
    {
      if(IS_FIRST_CAPTURED == 0)
      {
        IS_FIRST_CAPTURED = 1;
        IC_Val1 = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);
        __HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
      }
      else if(IS_FIRST_CAPTURED == 1)
      {
        IS_FIRST_CAPTURED = 0;
        IC_Val2 = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);
        __HAL_TIM_SET_COUNTER(htim,0);

        if(IC_Val1 < IC_Val2)
        {
          Difference = IC_Val2 - IC_Val1;
        }
        else if(IC_Val1 > IC_Val2)
        {
          Difference = 0xffff - IC_Val1 + IC_Val2;
        }
        Distance = Difference * 340 / 2 / 10000;  // unit:cm

        __HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
        __HAL_TIM_DISABLE_IT(&htim3,TIM_IT_CC1);
      }
    }
  }
}

#endif