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
#include "navigation_public.h"
#include "communication_public.h"
#include "myjson.h"
#include "math.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Constants                                                         */
/* ************************************************************************** */
/* ************************************************************************** */
#define INVERTED_X_AXIS 0
    
#define MOTOR_2_FORWARDS 0
#define MOTOR_2_BACKWARDS 1
#define MOTOR_1_FORWARDS 0
#define MOTOR_1_BACKWARDS 1

//IDs used to identify data types
#define MOTOR_SPEED_ID 0x3c//60
#define MOTOR_DIRECTION_ID 0x66//102
#define MOTOR_DISTANCE_ID 0xc3//195
#define MOTOR_ANGLE_ID 0xe7//231
    
//Constants used in testing code
#define TEST_SPEED_SERVER_ID ((unsigned char) 'q')
#define TEST_ANGLE_DISTANCE_SERVER_ID ((unsigned char) 'x')
#define TEST_COMM_ID_IDX 2
#define TEST_COMM_SPEED_IDX 0
#define TEST_NAV_ID_IDX 1
#define TEST_NAV_DATA_IDX 0
#define MOTOR_PATHFINDING_INTEGRATION_TESTING 1
    
//Useful constants
#define ROVER_DIRECTION_LEFT 0x0003
#define ROVER_DIRECTION_RIGHT 0x000c
#define ROVER_DIRECTION_FORWARDS 0x0030
#define ROVER_DIRECTION_BACKWARDS 0x00c0
#define ROVER_SPEED_STRAIGHT 45
#define ROVER_SPEED_TURNING 38
#define ROVER_SPEED_MEDIUM 40
#define ROVER_SPEED_SLOW 15
#define ROVER_SPEED_SLOW_TURNING 10
#define ROVER_SPEED_STOPPED 0
#define ROVER_TICKS_REMAINING_MAX 0x7fffffff
#define ROVER_TICKS_REMAINING_SLOW 400
#define ROVER_TICKS_REMAINING_NONE 0
#define CALCULATE_IN_INCHES ((bool) true)
#define CALCULATE_IN_CENTIMETERS ((bool) false)

// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************
    
//These functions are used to interface with the location and orientation of the rover
void SetOrientation(float orientation);
float GetOrientation();
void SetLocationX(float x);
float GetLocationX();
void SetLocationY(float y);
float GetLocationY();
    
//These turn the motors on/off, and make them go forwards/backwards
void Motor1SetDirection(int direction);
void Motor2SetDirection(int direction);
void Motor1SetPWM(int pwm);
void Motor2SetPWM(int pwm);

//This function handles distance remaining and controls the speed of the rover
void HandleDistanceRemaining(int *desiredSpeed, int *ticksRemaining, int changeInSpeed);

//This function handles position and orientation updates
//The orientation is returned as a degree value between -180 and 180
void HandlePositionAndOrientation(int speed, int direction, bool inOrCm);

//These functions handle setting the direction of the rover
void SetDirectionClockwise();
void SetDirectionCounterclockwise();
void SetDirectionForwards();
void SetDirectionBackwards();
//This function returns the current direction of the rover
int GetMotorDirection();

//This function takes a value from 0 to 255
//The value represents half of an angle value, ex: 65 = 130 degrees
//Returns an angle value from -180 to 180
int ConvertJSONToAngle(unsigned char jsonAngle);

//These functions are used for converting between encoder ticks and ideal rotation degrees
int deg2tick(int deg);
int tick2deg(int ticks);
int deg2tickF(float deg);
float tick2degF(int ticks);

//These functions are used for converting between encoder ticks and distance in inches
int in2tick(int inches);
int tick2in(int ticks);
int in2tickF(float inches);
float tick2inF(int ticks);

//These functions are used for converting between encoder ticks and distance in inches
int cm2tick(int centimeters);
int tick2cm(int ticks);
int cm2tickF(float centimeters);
float tick2cmF(int ticks);

//These functions return distance and angle in ticks
//This function calculates the distance in inches or centimeters between two points
int CalculateDistanceFromPoints(int x1, int y1, int x2, int y2, bool inOrCm);
//This function calculates the angle between a vector defined by two points and 0 degrees
//The return value is in degrees
//int CalculateAngleFromPoints(int x1, int y1, int x2, int y2, float orientation);
int CalculateAngleFromPoints(int x1, int y1, int x2, int y2);

//This function adjusts the angle to rotate by the current angle
int AdjustAngleToRotate(int angleTicks, float orientation);

//These functions calculate the amount of ticks needed to rotate or travel a certain distance
int CalculateTicksToTravel(unsigned char distance, bool inOrCm);
int CalculateTicksToRotate(unsigned char angle);

//Given a value between 0 and 25, calculate the PWM
//count is a number from 0 to 24
//The duty cycle is (val*4)%
int GetPWMFromValue(unsigned int val, unsigned int count);

//Generate a PWM signal based on the error
//Send a 0 - 25 value to the PWM
int PID1(unsigned int setpoint, unsigned int actual);
int PID2(unsigned int setpoint, unsigned int actual);

//These functions abstract all of the code used in server testing
void motorTestCommReceive(unsigned char receivemsg[COMM_QUEUE_BUFFER_SIZE]);
void motorTestCommSend(unsigned char receivemsg[COMM_QUEUE_BUFFER_SIZE]);
void motorTestNavReceive(unsigned char receivemsg[NAV_QUEUE_BUFFER_SIZE], int *currentSpeed, int *ticksRemaining);
void motorTestNavSendSpeed(int speed);


/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */

/* *****************************************************************************
 End of File
 */
