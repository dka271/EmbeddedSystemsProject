/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef MOTOR_H    /* Guard against multiple inclusion */
#define MOTOR_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include "system_config.h"
#include "system_definitions.h"
#include "system/system.h"
#include "system/clk/sys_clk.h"
#include "driver/usart/drv_usart.h"
#include "system/devcon/sys_devcon.h"
#include "peripheral/peripheral.h"

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */
#define MOTOR_2_FORWARDS 0
#define MOTOR_2_BACKWARDS 1
#define MOTOR_1_FORWARDS 0
#define MOTOR_1_BACKWARDS 1

#define NAV_MOTOR_SPEED_ID 0x3c

    // *****************************************************************************
    // *****************************************************************************
    // Section: Data Types
    // *****************************************************************************
    // *****************************************************************************



    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************
    
    void Motor1SetDirection(int direction);
    void Motor2SetDirection(int direction);
    void Motor1SetPWM(int pwm);
    void Motor2SetPWM(int pwm);
    
    

//Given a value between 0 and 20, calculate the PWM
//count is a number from 0 to 19
//The duty cycle is (val*5)%
int GetPWMFromValue(unsigned int val, unsigned int count);

//Generate a PWM signal based on the error
//Send a 0 - 20 value to the PWM
int PID1(unsigned int setpoint, unsigned int actual);
int PID2(unsigned int setpoint, unsigned int actual);



    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */

/* *****************************************************************************
 End of File
 */
