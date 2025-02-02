/**
 * @file AppConfig.h
 * @author Ed Z (zouhetai@outlook.com)
 * @brief app config file to enable and disable certian modules
 * @version 0.1
 * @date 2023-09-07
 *
 * @copyright RM2024 Enterprize Copyright (c) 2023
 *
 */
// clang-format off

#pragma once

// dji motor
#define USE_DJI_MOTOR TRUE
    #if USE_DJI_MOTOR
        #ifndef DJI_MOTOR_CAN
        #define DJI_MOTOR_CAN hcan
        #endif
    #endif

#define USE_DR16 TRUE
    #if USE_DR16
        #ifndef DR16_UART
        #define DR16_UART huart1
        #endif
    #endif

#define USE_BUZZER FALSE
    #if USE_BUZZER
        #ifndef BUZZER_TIM_CLOCK
        #define BUZZER_TIM_CLOCK 72000000
        #endif

        #ifndef BUZZER_TIM
        #define BUZZER_TIM htim1
        #endif

        #ifndef BUZZER_TIM_CHANNEL
        #define BUZZER_TIM_CHANNEL TIM_CHANNEL_1
        #endif

        #ifndef BUZZER_QUEUE_LENGTH
        #define BUZZER_QUEUE_LENGTH 16
        #endif
    #endif

// PID
#define USE_PID TRUE

// TR
#define IS_TR TRUE

// AR
#define IS_AR TRUE

// HCSR04
#define USE_HCSR04 TRUE

// LineTracker
#define USE_LINETRACKER TRUE

// MG996
#define USE_MG996R TRUE

// HC06
#define USE_HC05 TRUE

    
