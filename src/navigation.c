/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    navigation.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

#include "navigation.h"
#include "navigation_public.h"
#include "ms2test.h"
#include "mapping_public.h"

NAVIGATION_DATA navigationData;

static QueueHandle_t navQueue;

void NAVIGATION_Initialize(void) {
    /* Place the App state machine in its initial state. */
    navigationData.state = NAVIGATION_STATE_INIT;

    //Initialize the navigation queue
    navQueue = xQueueCreate(20, sizeof (unsigned char[NAV_QUEUE_BUFFER_SIZE]));
    if (navQueue == 0) {
        dbgPauseAll();
    }
}

void navSendMsgFromISR(unsigned char msg[NAV_QUEUE_BUFFER_SIZE]) {
    BaseType_t xHigherPriorityTaskWoken = pdTRUE; //pdFALSE;
    xQueueSendToBackFromISR(navQueue, msg, NULL);
}

void navSendMsg(unsigned char msg[NAV_QUEUE_BUFFER_SIZE]) {
    BaseType_t xHigherPriorityTaskWoken = pdTRUE; //pdFALSE;
    xQueueSendToBack(navQueue, msg, portMAX_DELAY);
}

unsigned char navCalculateChecksum(unsigned char msg[NAV_QUEUE_BUFFER_SIZE]) {
    unsigned char sum = 0;
    unsigned int i;
    for (i = 0; i < NAV_QUEUE_BUFFER_SIZE - 1; i++) {
        sum += msg[i];
    }
    return sum;
}


//Movement Control stuff
unsigned int moveAmount[100];
unsigned char moveType[100];
unsigned int moveFirstIdx = 0;
unsigned int moveCurrentIdx = 0;
unsigned int moveLastIdx = 0;
unsigned int moveGoalIdx = 0xff;
unsigned int moveMaxIdx = 99;
int roverStopped = 0;
unsigned int desiredSpeed = ROVER_SPEED_STOPPED;
int ticksRemaining = ROVER_TICKS_REMAINING_NONE;
//Line Sensing stuff
int ignoringTape = 0;
int ignoreTapeCount = 0;
int ignoreTapeMax = 40;
bool cs1OnTape = false;
bool cs2OnTape = false;
bool cs3OnTape = false;
bool cs4OnTape = false;
int orientAttempts = 0;
//Variables to be used for navigation
unsigned char receivemsg[NAV_QUEUE_BUFFER_SIZE];
unsigned int previousValue1 = 0;
unsigned int speed1;
unsigned int previousValue2 = 0;
unsigned int speed2;
unsigned int pwmCount = 0;
int m1PID;
int m2PID;
unsigned char oriented = 0;
unsigned char movingOriented = 0;
unsigned char orientBackwards = 1;
unsigned char numberOfLines = 0;
unsigned char currDirectionBool = 0;
unsigned char expectingBack = 1;
unsigned char midLine0 = 0;
unsigned char midLine1 = 0;
unsigned char farLine = 0;
unsigned char midLine0Set = 0;
unsigned char midLine1Set = 0;
unsigned char farLineSet = 0;
unsigned char readyToSample = 0;

// *****************************************************************************
//Movement API functions

//Clears any path that is currently in the queue

void ResetMovementQueue() {
    moveCurrentIdx = 0;
    moveLastIdx = 0;
    moveGoalIdx = 0xff;
}

//Stops the current movement of the rover

void StopMovement() {
    ticksRemaining = 0;
    desiredSpeed = ROVER_SPEED_STOPPED;
}

//Adds a movement to the queue

void AddMovement(int tickAmount, int direction) {
    moveAmount[moveLastIdx] = tickAmount;
    moveType[moveLastIdx] = direction;
    moveLastIdx++;
}

//Sets the goal to the movement that was just added
unsigned char navStartMapping(){
    return readyToSample;
}


void SetMovementGoal() {
    moveGoalIdx = moveLastIdx;
}

//This function handles reading from the movement queue and starting movements

bool HandleMovementQueue() {
    bool toReturn = false;
    if (speed2 == 0) {
        roverStopped++;
    } else {
        roverStopped = 0;
    }
    if (roverStopped >= 10 && moveLastIdx > moveCurrentIdx) {
        //There is a command, and the rover is not moving
        //Interpret the command and control the rover
        if (moveType[moveCurrentIdx] == ROVER_DIRECTION_LEFT) {
            //Handle a left turn
            SetDirectionCounterclockwise();
            ticksRemaining = moveAmount[moveCurrentIdx];
            if (ticksRemaining < 0) {
                ticksRemaining *= -1;
                SetDirectionClockwise();
            }
            int minRotation = deg2tickF(30);
            if (ticksRemaining >= minRotation) {
                //Normal turning speed
                desiredSpeed = ROVER_SPEED_TURNING;
            } else {
                //Slow turning speed
                desiredSpeed = ROVER_SPEED_SLOW_TURNING;
                ticksRemaining += 30;
            }
        } else if (moveType[moveCurrentIdx] == ROVER_DIRECTION_RIGHT) {
            //Handle a right turn
            SetDirectionClockwise();
            ticksRemaining = moveAmount[moveCurrentIdx];
            if (ticksRemaining < 0) {
                ticksRemaining *= -1;
                SetDirectionCounterclockwise();
            }
            int minRotation = deg2tickF(30);
            if (ticksRemaining >= minRotation) {
                //Normal turning speed
                desiredSpeed = ROVER_SPEED_TURNING;
            } else {
                //Slow turning speed
                desiredSpeed = ROVER_SPEED_SLOW_TURNING;
                ticksRemaining += 30;
            }
        } else if (moveType[moveCurrentIdx] == ROVER_DIRECTION_FORWARDS) {
            //Handle forward movement
            SetDirectionForwards();
            ticksRemaining = moveAmount[moveCurrentIdx];
            int minMove = in2tick(2);
            if (ticksRemaining <= minMove) {
                desiredSpeed = ROVER_SPEED_SLOW;
            } else {
                desiredSpeed = ROVER_SPEED_STRAIGHT;
            }
        } else if (moveType[moveCurrentIdx] == ROVER_DIRECTION_BACKWARDS) {
            //Handle backward movement
            SetDirectionBackwards();
            ticksRemaining = moveAmount[moveCurrentIdx];
            int minMove = in2tick(2);
            if (ticksRemaining <= minMove) {
                desiredSpeed = ROVER_SPEED_SLOW;
            } else {
                desiredSpeed = ROVER_SPEED_STRAIGHT;
            }
        }
        moveCurrentIdx++;
        roverStopped = 0;
    } else if (roverStopped > 10 && moveCurrentIdx == moveGoalIdx) {
        //Destination has been reached, reset the command queue
        ResetMovementQueue();
        toReturn = true;
    }
    return toReturn;
}

//This function handles movement control when the side color sensor enters or leaves tape

void HandleSideColorSensor(int colorSensorId) {
    if (ignoringTape){
        return;
    }
    switch (movementState) {
        case (STATE_WAITING_FOR_GAME_START):
        {
            break;
        }
        case (STATE_LOOKING_FOR_FLAG):
        {
            break;
        }
        case (STATE_ORIENTING):
        {
            //            orientAttempts++;
            //            if (!cs3OnTape){
            //The side color sensor is not on tape
            if (cs1OnTape && !cs2OnTape) {

            } else if (cs1OnTape && cs2OnTape) {
                int dir = GetMotorDirection();
                ResetMovementQueue();
                StopMovement();
                switch (dir) {
                    case (ROVER_DIRECTION_LEFT):
                    {
                        SetDirectionClockwise();
                        break;
                    }
                    case (ROVER_DIRECTION_RIGHT):
                    {
                        SetDirectionCounterclockwise();
                        break;
                    }
                    case (ROVER_DIRECTION_FORWARDS):
                    {
                        SetDirectionBackwards();
                        break;
                    }
                    case (ROVER_DIRECTION_BACKWARDS):
                    {
                        SetDirectionForwards();
                        break;
                    }
                }

            } else if (!cs1OnTape && cs2OnTape) {

            } else if (!cs1OnTape && !cs2OnTape) {

            }
            //            }else{
            //The side color sensor is on tape
            //                if (cs1OnTape && !cs2OnTape){
            //                    
            //                }else if (cs1OnTape && cs2OnTape){
            //                    
            //                }else if (!cs1OnTape && cs2OnTape){
            //                    
            //                }else if (!cs1OnTape && !cs2OnTape){
            //                    //The only way this can happen is if the corner sensors are on opposite sides of the line
            //                    //Undo the movement that got us here
            //                }
            //            }
            break;
        }
        case (STATE_MOVING):
        {
            
            if (cs1OnTape && cs2OnTape) {
                if (cs3OnTape && !cs4OnTape) {
                    if(expectingBack){
                        numberOfLines++;
                        expectingBack = 0;
                    }
                    if (orientBackwards == 1) {
                        ignoringTape = 1;
                        ignoreTapeCount = 30;
                        SetDirectionForwards();
                        numberOfLines = 0;
                        ResetMovementQueue();
                        StopMovement();
                        AddMovement(in2tick(50), ROVER_DIRECTION_FORWARDS);
                        currDirectionBool = 0;
                        SetMovementGoal();
                        orientBackwards = 0;
                        SetLocationY(3);
                        
                        //setbooltrue
                        readyToSample = 1;
                        
                        
                    }


                    if (numberOfLines >= 5) {
                        ResetMovementQueue();
                        StopMovement();
                        int dir = GetMotorDirection();
                        switch (dir) {
                            case (ROVER_DIRECTION_LEFT):
                            {
                                AddMovement(deg2tick(25), ROVER_DIRECTION_RIGHT);
                                SetMovementGoal();
                                break;
                            }
                            case (ROVER_DIRECTION_RIGHT):
                            {
                                AddMovement(deg2tick(25), ROVER_DIRECTION_LEFT);
                                SetMovementGoal();
                                break;
                            }
                            case (ROVER_DIRECTION_FORWARDS):
                            {
                                AddMovement(cm2tick(10), ROVER_DIRECTION_BACKWARDS);
                                SetMovementGoal();
                                break;
                            }
                            case (ROVER_DIRECTION_BACKWARDS):
                            {
                                AddMovement(cm2tick(10), ROVER_DIRECTION_FORWARDS);
                                SetMovementGoal();
                                break;
                            }
                        }

                        if (currDirectionBool == 0) {
                            SetDirectionBackwards();
                            numberOfLines = 0;
                            ResetMovementQueue();
                            StopMovement();
                            AddMovement(in2tick(50), ROVER_DIRECTION_BACKWARDS);
                            currDirectionBool = 1;
                            SetMovementGoal();
                        } else {
                            SetDirectionForwards();
                            numberOfLines = 0;
                            ResetMovementQueue();
                            StopMovement();
                            AddMovement(in2tick(50), ROVER_DIRECTION_FORWARDS);
                            currDirectionBool = 0;
                            SetMovementGoal();
                        }


                    }

                } else if (!cs3OnTape && cs4OnTape) {
                    
                    if(!expectingBack){
                        numberOfLines++;
                        expectingBack = 1;
                        if(!midLine0Set){
                            midLine0 = GetLocationY() + 3;
                            midLine0Set = 1;
                        }
                        else if(midLine0Set && !midLine1Set){
                            midLine1 = GetLocationY() + 3;
                            midLine1Set = 1;
                        }
                        else if(midLine0Set && midLine1Set && !farLineSet){
                            farLine = GetLocationY() + 3;
                            farLineSet = 1;
                        }
                    }
                    if (numberOfLines >= 5) {
                        int dir = GetMotorDirection();
                        switch (dir) {
                            case (ROVER_DIRECTION_LEFT):
                            {
                                AddMovement(deg2tick(25), ROVER_DIRECTION_RIGHT);
                                SetMovementGoal();
                                break;
                            }
                            case (ROVER_DIRECTION_RIGHT):
                            {
                                AddMovement(deg2tick(25), ROVER_DIRECTION_LEFT);
                                SetMovementGoal();
                                break;
                            }
                            case (ROVER_DIRECTION_FORWARDS):
                            {
                                AddMovement(cm2tick(10), ROVER_DIRECTION_BACKWARDS);
                                SetMovementGoal();
                                break;
                            }
                            case (ROVER_DIRECTION_BACKWARDS):
                            {
                                AddMovement(cm2tick(10), ROVER_DIRECTION_FORWARDS);
                                SetMovementGoal();
                                break;
                            }
                        }

                        if (currDirectionBool == 0) {
                            SetDirectionBackwards();
                            numberOfLines = 0;
                            ResetMovementQueue();
                            StopMovement();
                            AddMovement(in2tick(50), ROVER_DIRECTION_BACKWARDS);
                            currDirectionBool = 1;
                            SetMovementGoal();
                        } else {
                            SetDirectionForwards();
                            numberOfLines = 0;
                            ResetMovementQueue();
                            StopMovement();
                            AddMovement(in2tick(50), ROVER_DIRECTION_FORWARDS);
                            currDirectionBool = 0;
                            SetMovementGoal();
                        }


                    }

                }


            }


            break;
        }
        case(STATE_REORIENT):{
            if (cs1OnTape && cs2OnTape) {
                ResetMovementQueue();
                StopMovement();
                if (currDirectionBool == 0){
                    AddMovement(in2tick(50), ROVER_DIRECTION_FORWARDS);
                    SetMovementGoal();
                }else{
                    AddMovement(in2tick(50), ROVER_DIRECTION_BACKWARDS);
                    SetMovementGoal();
                }
                movementState = STATE_MOVING;
            }
            else if (cs2OnTape && !cs1OnTape && cs4OnTape){
                ResetMovementQueue();
                StopMovement();
                AddMovement(cm2tick(1), ROVER_DIRECTION_BACKWARDS);
                AddMovement(deg2tick(20), ROVER_DIRECTION_LEFT);
                AddMovement(cm2tick(1), ROVER_DIRECTION_BACKWARDS);
                AddMovement(deg2tick(20), ROVER_DIRECTION_LEFT);
                AddMovement(cm2tick(1), ROVER_DIRECTION_BACKWARDS);
                AddMovement(deg2tick(20), ROVER_DIRECTION_LEFT);
                SetMovementGoal();
            }
            else if(!cs2OnTape && cs1OnTape && cs3OnTape){
                ResetMovementQueue();
                StopMovement();
                AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
                AddMovement(deg2tick(20), ROVER_DIRECTION_RIGHT);
                AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
                AddMovement(deg2tick(20), ROVER_DIRECTION_RIGHT);
                AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
                AddMovement(deg2tick(20), ROVER_DIRECTION_RIGHT);
                SetMovementGoal();
            }
            
            break;
        }
    }
}

//This function handles movement control when a corner color sensor enters or leaves tape

void HandleCornerColorSensor(int colorSensorId) {
    bool csMeOnTape;
    bool csOtherOnTape;
    int turn1Direction;
    int turn2Direction;
    int move1Direction;
    int move2Direction;
    if (colorSensorId == NAV_COLOR_SENSOR_1_ID_SENSOR) {
        //This is the front left color sensor
        csMeOnTape = cs1OnTape;
        csOtherOnTape = cs2OnTape;
        turn1Direction = ROVER_DIRECTION_LEFT;
        turn2Direction = ROVER_DIRECTION_RIGHT;
        move1Direction = ROVER_DIRECTION_FORWARDS;
        move2Direction = ROVER_DIRECTION_BACKWARDS;
    } else if (colorSensorId == NAV_COLOR_SENSOR_2_ID_SENSOR) {
        //This is the back left color sensor
        csMeOnTape = cs2OnTape;
        csOtherOnTape = cs1OnTape;
        turn2Direction = ROVER_DIRECTION_LEFT;
        turn1Direction = ROVER_DIRECTION_RIGHT;
        move2Direction = ROVER_DIRECTION_FORWARDS;
        move1Direction = ROVER_DIRECTION_BACKWARDS;
    } else {
        //The color sensor ID is unknown
        return;
    }

    //Do nothing if we are ignoring tape
    if (ignoringTape) {
        return;
    }

    switch (movementState) {
        case (STATE_WAITING_FOR_GAME_START):
        {
            if (csMeOnTape && !csOtherOnTape) {
                //This sensor just got on tape, and the other isn't on tape
                //Move off of this tape
                ResetMovementQueue();
                StopMovement();
                AddMovement(cm2tick(2), move2Direction);
                SetMovementGoal();
            } else if (csMeOnTape && csOtherOnTape) {
                //Both corner sensors are on tape
                //Rotate so this isn't on tape
                ResetMovementQueue();
                StopMovement();
                AddMovement(deg2tick(15), turn1Direction);
                SetMovementGoal();
            } else if (!csMeOnTape && csOtherOnTape) {
                //This sensor just got off tape, and the other is on tape
                //Move off the tape
                ResetMovementQueue();
                StopMovement();
                AddMovement(cm2tick(2), move1Direction);
                SetMovementGoal();
            } else if (!csMeOnTape && !csOtherOnTape) {
                //This just got off tape, and the other isn't on tape
                //Kill any path, and stop the rover from moving
                ResetMovementQueue();
                StopMovement();
            }
            break;
        }
        case (STATE_LOOKING_FOR_FLAG):
        {
            break;
        }
        case (STATE_ORIENTING):
        {
            orientAttempts++;
            if ((orientAttempts > 20) && (cs1OnTape || cs2OnTape || !cs3OnTape)) {
                //This orientation is failing, back up and try again
                ResetMovementQueue();
                StopMovement();
                AddMovement(deg2tick(180), ROVER_DIRECTION_RIGHT);
                AddMovement(cm2tick(9), ROVER_DIRECTION_FORWARDS);
                AddMovement(deg2tick(160), ROVER_DIRECTION_LEFT);
                AddMovement(cm2tick(20), ROVER_DIRECTION_FORWARDS);
                SetMovementGoal();
                ignoringTape = 1;
                ignoreTapeCount = -20;
            } else { //if (!cs3OnTape){
                //The side color sensor is not on tape
                if (csMeOnTape && !csOtherOnTape) {
                    //This sensor just got on tape, and the other isn't on tape
                    //Rotate to try to get the side sensor on tape
                    ResetMovementQueue();
                    StopMovement();
                    AddMovement(cm2tick(2), move1Direction);
                    AddMovement(cm2tick(2), move1Direction);
                    AddMovement(deg2tick(COLOR_SENSOR_ANGLE), turn2Direction);
                    SetMovementGoal();
                } else if (csMeOnTape && csOtherOnTape) {
                    //Both corner sensors are on tape
                    //Rotate so this isn't on tape
                    //STATE WE WANT
                    ResetMovementQueue();
                    StopMovement();
                    movingOriented = 1;
                    movementState = STATE_MOVING;
                    SetOrientation(90);
                    switch (GetMotorDirection()) {
                            //Attempt to get the most accurate alignment
                        case (ROVER_DIRECTION_LEFT):
                        {
                            SetDirectionClockwise();
                            break;
                        }
                        case (ROVER_DIRECTION_RIGHT):
                        {
                            SetDirectionCounterclockwise();
                            break;
                        }
                        case (ROVER_DIRECTION_FORWARDS):
                        {
                            SetDirectionBackwards();
                            break;
                        }
                        case (ROVER_DIRECTION_BACKWARDS):
                        {
                            SetDirectionForwards();
                            break;
                        }
                    }
                    //                    AddMovement()
                    //                    AddMovement(deg2tick(15), turn1Direction);

                    //                    SetMovementGoal();
                } else if (!csMeOnTape && csOtherOnTape) {
                    //This sensor just got off tape, and the other is on tape
                    //Do nothing
                    ResetMovementQueue();
                    StopMovement();
                    AddMovement(deg2tick(COLOR_SENSOR_ANGLE), turn2Direction);
                    SetMovementGoal();
                } else if (!csMeOnTape && !csOtherOnTape) {
                    //Everything is off tape
                    //Turn towards the tape
                    ResetMovementQueue();
                    StopMovement();
//                    AddMovement(cm2tickF(1.5), move2Direction);
                    AddMovement(deg2tick(20), turn2Direction);
                    AddMovement(deg2tick(20), turn2Direction);
                    AddMovement(deg2tick(20), turn2Direction);
                    AddMovement(deg2tick(20), turn2Direction);
                    SetMovementGoal();

                }
            }
            break;
        }
        case (STATE_MOVING):
        {
            
            int dir = GetMotorDirection();
            ResetMovementQueue();
            StopMovement();
//            switch (dir) {
//                case (ROVER_DIRECTION_LEFT):
//                {
//                    AddMovement(deg2tick(25), ROVER_DIRECTION_RIGHT);
//                    SetMovementGoal();
//                    break;
//                }
//                case (ROVER_DIRECTION_RIGHT):
//                {
//                    AddMovement(deg2tick(25), ROVER_DIRECTION_LEFT);
//                    SetMovementGoal();
//                    break;
//                }
//                case (ROVER_DIRECTION_FORWARDS):
//                {
//                    AddMovement(cm2tick(10), ROVER_DIRECTION_BACKWARDS);
//                    SetMovementGoal();
//                    break;
//                }
//                case (ROVER_DIRECTION_BACKWARDS):
//                {
//                    AddMovement(cm2tick(10), ROVER_DIRECTION_FORWARDS);
//                    SetMovementGoal();
//                    break;
//                }
//            }

            if (!csMeOnTape && csOtherOnTape) {
                ResetMovementQueue();
                StopMovement();
                AddMovement(cm2tick(1), move2Direction);
                AddMovement(deg2tick(20), turn2Direction);
                AddMovement(cm2tick(1), move2Direction);
                AddMovement(deg2tick(20), turn2Direction);
                AddMovement(cm2tick(1), move2Direction);
                AddMovement(deg2tick(20), turn2Direction);
                SetMovementGoal();
                movementState = STATE_REORIENT;
            } else if (csMeOnTape && !csOtherOnTape) {
                ResetMovementQueue();
                StopMovement();
                AddMovement(cm2tick(1), move1Direction);
                AddMovement(deg2tick(20), turn1Direction);
                AddMovement(cm2tick(1), move1Direction);
                AddMovement(deg2tick(20), turn1Direction);
                AddMovement(cm2tick(1), move1Direction);
                AddMovement(deg2tick(20), turn1Direction);
                SetMovementGoal();
                 movementState = STATE_REORIENT;
            }
            else if(!csMeOnTape && !csOtherOnTape){
                movementState = STATE_REORIENT;
            }


            break;
        }
        case (STATE_REORIENT):
        {

            if (csMeOnTape && csOtherOnTape) {
                
                int dir = GetMotorDirection();
                ResetMovementQueue();
                StopMovement();
                
//                if(GetOrientation() - 90 < 7 && GetOrientation() - 90 > -7 ){
                    movementState = STATE_MOVING;
                    if (currDirectionBool == 0){
                        if(dir == ROVER_DIRECTION_LEFT){
                            AddMovement(deg2tick(10), ROVER_DIRECTION_LEFT);
                            SetOrientation(90);
                        }
                        else if(dir == ROVER_DIRECTION_RIGHT){
                            AddMovement(deg2tick(10), ROVER_DIRECTION_RIGHT);
                            SetOrientation(90);
                        }
                        AddMovement(in2tick(50), ROVER_DIRECTION_FORWARDS);
                        SetMovementGoal();
                    }else{
                        if(dir == ROVER_DIRECTION_LEFT){
                            AddMovement(deg2tick(10), ROVER_DIRECTION_LEFT);
                        }
                        else if(dir == ROVER_DIRECTION_RIGHT){
                            AddMovement(deg2tick(10), ROVER_DIRECTION_RIGHT);
                        }
                        AddMovement(in2tick(50), ROVER_DIRECTION_BACKWARDS);
                        SetMovementGoal();
                    }
//                }
//                else if(GetOrientation() - 90 > 0){
//                    AddMovement(deg2tick(20), ROVER_DIRECTION_LEFT);
//                    AddMovement(deg2tick(10), ROVER_DIRECTION_LEFT);
//                    AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
//                    AddMovement(deg2tick(20), ROVER_DIRECTION_LEFT);
//                    AddMovement(deg2tick(10), ROVER_DIRECTION_LEFT);
//                    AddMovement(cm2tick(1), ROVER_DIRECTION_BACKWARDS);
//                    AddMovement(deg2tick(20), ROVER_DIRECTION_LEFT);
//                    AddMovement(deg2tick(10), ROVER_DIRECTION_LEFT);
//                    AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
//                    AddMovement(deg2tick(20), ROVER_DIRECTION_LEFT);
//                    AddMovement(deg2tick(10), ROVER_DIRECTION_LEFT);
//                    AddMovement(cm2tick(1), ROVER_DIRECTION_BACKWARDS);
//                    SetMovementGoal();
//                    
//                }
//                else{
//                    AddMovement(deg2tick(20), ROVER_DIRECTION_RIGHT);
//                    AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
//                    AddMovement(deg2tick(20), ROVER_DIRECTION_RIGHT);
//                    AddMovement(deg2tick(10), ROVER_DIRECTION_RIGHT);
//                    AddMovement(cm2tick(1), ROVER_DIRECTION_BACKWARDS);
//                    AddMovement(deg2tick(20), ROVER_DIRECTION_RIGHT);
//                    AddMovement(deg2tick(10), ROVER_DIRECTION_RIGHT);
//                    AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
//                    AddMovement(deg2tick(20), ROVER_DIRECTION_RIGHT);
//                    AddMovement(deg2tick(10), ROVER_DIRECTION_RIGHT);
//                    AddMovement(cm2tick(1), ROVER_DIRECTION_BACKWARDS);
//                    SetMovementGoal();
//                
//                }            
                
            }
            else if (!csMeOnTape && csOtherOnTape) {
                ResetMovementQueue();
                StopMovement();
//                AddMovement(cm2tick(1), move2Direction);
//                AddMovement(deg2tick(20), turn2Direction);
//                AddMovement(cm2tick(1), move2Direction);
//                AddMovement(deg2tick(20), turn2Direction);
//                AddMovement(cm2tick(1), move2Direction);
//                AddMovement(deg2tick(20), turn2Direction);
                
                
                 if(currDirectionBool == 0){
                AddMovement(deg2tick(20), turn1Direction);
                AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
                AddMovement(deg2tick(20), turn2Direction);
                AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
                AddMovement(deg2tick(20), turn1Direction);
                AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
                AddMovement(deg2tick(20), turn2Direction);
                AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
//                AddMovement(deg2tick(20), turn1Direction);
//                AddMovement(deg2tick(20), turn1Direction);
//                AddMovement(deg2tick(20), turn1Direction);
                    
                }
                
                else if(currDirectionBool == 1){
                AddMovement(deg2tick(20), turn1Direction);
                AddMovement(cm2tick(1), ROVER_DIRECTION_BACKWARDS);
                AddMovement(deg2tick(20), turn2Direction);
                AddMovement(cm2tick(1), ROVER_DIRECTION_BACKWARDS);
//                AddMovement(deg2tick(20), turn2Direction);
                AddMovement(deg2tick(20), turn1Direction);
                AddMovement(cm2tick(1), ROVER_DIRECTION_BACKWARDS);
                AddMovement(deg2tick(20), turn2Direction);
                AddMovement(cm2tick(1), ROVER_DIRECTION_BACKWARDS);
                    
                }
                
                
//                AddMovement(deg2tick(20), turn2Direction);
//                AddMovement(deg2tick(20), turn1Direction);
//                AddMovement(deg2tick(20), turn1Direction);
//                AddMovement(deg2tick(20), turn1Direction);
//                AddMovement(deg2tick(20), turn2Direction);
//                AddMovement(deg2tick(20), turn2Direction);
//                AddMovement(deg2tick(20), turn2Direction);
//                AddMovement(deg2tick(20), turn2Direction);
//                AddMovement(deg2tick(20), turn2Direction);
                SetMovementGoal();
            } else if (csMeOnTape && !csOtherOnTape) {
                ResetMovementQueue();
                StopMovement();
//                AddMovement(cm2tick(1), move1Direction);
//                AddMovement(deg2tick(20), turn1Direction);
//                AddMovement(cm2tick(1), move1Direction);
//                AddMovement(deg2tick(20), turn1Direction);
//                AddMovement(cm2tick(1), move1Direction);
//                AddMovement(deg2tick(20), turn1Direction);

                if(currDirectionBool == 0){
                AddMovement(deg2tick(20), turn1Direction);
                AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
                AddMovement(deg2tick(20), turn2Direction);
                AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
//                AddMovement(deg2tick(20), turn2Direction);
                AddMovement(deg2tick(20), turn1Direction);
                AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
                AddMovement(deg2tick(20), turn2Direction);
                AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
//                AddMovement(deg2tick(20), turn1Direction);
//                AddMovement(deg2tick(20), turn1Direction);
//                AddMovement(deg2tick(20), turn1Direction);
                    
                }
                
                else if(currDirectionBool == 1){
                AddMovement(deg2tick(20), turn1Direction);
                AddMovement(cm2tick(1), ROVER_DIRECTION_BACKWARDS);
                AddMovement(deg2tick(20), turn2Direction);
                AddMovement(cm2tick(1), ROVER_DIRECTION_BACKWARDS);
//                AddMovement(deg2tick(20), turn2Direction);
                AddMovement(deg2tick(20), turn1Direction);
                AddMovement(cm2tick(1), ROVER_DIRECTION_BACKWARDS);
                AddMovement(deg2tick(20), turn2Direction);
                AddMovement(cm2tick(1), ROVER_DIRECTION_BACKWARDS);
                    
                }


//                AddMovement(deg2tick(20), turn1Direction);
//                AddMovement(deg2tick(20), turn2Direction);
//                AddMovement(deg2tick(20), turn2Direction);
//                AddMovement(deg2tick(20), turn2Direction);
//                AddMovement(deg2tick(20), turn1Direction);
//                AddMovement(deg2tick(20), turn1Direction);
//                AddMovement(deg2tick(20), turn1Direction);
//                AddMovement(deg2tick(20), turn1Direction);
                SetMovementGoal();
            } else if (!csMeOnTape && !csOtherOnTape) {
                if (GetMotorDirection() == ROVER_DIRECTION_LEFT) {
                    ResetMovementQueue();
                    StopMovement();
                    //check currentDirectionBool
                    AddMovement(cm2tick(1), ROVER_DIRECTION_BACKWARDS);
                    AddMovement(deg2tick(20), ROVER_DIRECTION_RIGHT);
                    AddMovement(cm2tick(1), ROVER_DIRECTION_BACKWARDS);
                    AddMovement(deg2tick(20), ROVER_DIRECTION_RIGHT);
                    AddMovement(cm2tick(1), ROVER_DIRECTION_BACKWARDS);
                    AddMovement(deg2tick(20), ROVER_DIRECTION_RIGHT);
                    SetMovementGoal();
                } else if (GetMotorDirection() == ROVER_DIRECTION_RIGHT) {
                    ResetMovementQueue();
                    StopMovement();
                    AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
                    AddMovement(deg2tick(20), ROVER_DIRECTION_LEFT);
                    AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
                    AddMovement(deg2tick(20), ROVER_DIRECTION_LEFT);
                    AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
                    AddMovement(deg2tick(20), ROVER_DIRECTION_LEFT);
                    SetMovementGoal();
                }
            }

            break;
        }
    }
}

/******************************************************************************
  Function:
    void NAVIGATION_Tasks ( void )

  Remarks:
    See prototype in navigation.h.
 */
void NAVIGATION_Tasks(void) {
    dbgOutputLoc(DBG_LOC_NAV_ENTER);
    //    SetDirectionForwards();
    SetDirectionClockwise();

    dbgOutputLoc(DBG_LOC_NAV_BEFORE_WHILE);
    movementState = STATE_LOOKING_FOR_FLAG;
    flagAngleFromStart = 0;

    //I2C Initialization Stuff
    //Open the I2C
    int i2cCount = -100; //Set this low so the color sensors are guaranteed to receive power by the time we start initializing them
    while (DRV_I2C_Status(sysObj.drvI2C0) != SYS_STATUS_READY) {
        //Wait for the I2C to be ready to be opened
        //FOR TESTING
        if (COLOR_SENSOR_SERVER_TESTING) {
            //            sprintf(testMsg, "Waiting for I2C 1...");
            //            commSendMsgToWifiQueue(testMsg);
        }
        //END FOR TESTING
    }
    DRV_HANDLE i2c1_handle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE);
    while (DRV_I2C_Status(sysObj.drvI2C1) != SYS_STATUS_READY) {
        //Wait for the I2C to be ready to be opened
        //FOR TESTING
        if (COLOR_SENSOR_SERVER_TESTING) {
            //            sprintf(testMsg, "Waiting for I2C 2...");
            //            commSendMsgToWifiQueue(testMsg);
        }
        //END FOR TESTING
    }
    DRV_HANDLE i2c2_handle = DRV_I2C_Open(DRV_I2C_INDEX_1, DRV_IO_INTENT_READWRITE);



    while (DRV_I2C_Status(sysObj.drvI2C2) != SYS_STATUS_READY) {
        //Wait for the I2C to be ready to be opened
        //FOR TESTING
        if (COLOR_SENSOR_SERVER_TESTING) {
            //            sprintf(testMsg, "Waiting for I2C 3...");
            //            commSendMsgToWifiQueue(testMsg);
        }
        //END FOR TESTING
    }
    DRV_HANDLE i2c3_handle = DRV_I2C_Open(DRV_I2C_INDEX_2, DRV_IO_INTENT_READWRITE);



    while (DRV_I2C_Status(sysObj.drvI2C3) != SYS_STATUS_READY) {
        //Wait for the I2C to be ready to be opened
        //FOR TESTING
        if (COLOR_SENSOR_SERVER_TESTING) {
            //            sprintf(testMsg, "Waiting for I2C 4...");
            //            commSendMsgToWifiQueue(testMsg);
        }
        //END FOR TESTING
    }
    DRV_HANDLE i2c4_handle = DRV_I2C_Open(DRV_I2C_INDEX_3, DRV_IO_INTENT_READWRITE);

    //Init the I2C state machine
    DRV_TCS_HandleColorSensor(NULL, COLOR_SENSOR_RESET_STATE_MACHINE);


    while (1) {
        //Block until a message is received
        dbgOutputLoc(DBG_LOC_NAV_BEFORE_RECEIVE);
        BaseType_t receiveCheck = xQueueReceive(navQueue, receivemsg, portMAX_DELAY);
        dbgOutputLoc(DBG_LOC_NAV_AFTER_RECEIVE);

        //Handle the message
        if (receiveCheck == pdTRUE) {
            //Convert the message into integer format
            unsigned int receivemsgint = receivemsg[0] | (receivemsg[1] << 8) | (receivemsg[2] << 16) | (receivemsg[3] << 24);
            //Get the message ID
            int msgId = (receivemsg[NAV_SOURCE_ID_IDX] & NAV_SOURCE_ID_MASK) >> NAV_SOURCE_ID_OFFSET;
            //Handle a specific message

            if (msgId == NAV_TIMER_COUNTER_3_ID_SENSOR) {
                //Motor 2 Encoder Message Handler
                speed2 = (receivemsgint & 0x0000ffff) - previousValue2;
                previousValue2 = receivemsgint & 0x0000ffff;

                //Handle path controls
                if (HandleMovementQueue()){
                    if (movementState == STATE_MOVING){
                        if (currDirectionBool == 0){
                            ResetMovementQueue();
                            StopMovement();
                            AddMovement(in2tick(50), ROVER_DIRECTION_FORWARDS);
                            SetMovementGoal();
                        }else{
                            ResetMovementQueue();
                            StopMovement();
                            AddMovement(in2tick(50), ROVER_DIRECTION_BACKWARDS);
                            SetMovementGoal();
                        }
                    }
                }

                //Handle remaining distance
                if (GetMotorDirection() == ROVER_DIRECTION_LEFT) {
                    HandleDistanceRemaining(&desiredSpeed, &ticksRemaining, speed2);

                    //Handle position and orientation
                    HandlePositionAndOrientation(speed2, GetMotorDirection(), CALCULATE_IN_CENTIMETERS);

                    //Handle Daniel's server testing
                    //                    motorTestNavSendSpeed(speed2);
                }

                //Spin around while looking for the flag
                if (oriented == 0 && movementState == STATE_LOOKING_FOR_FLAG) {
                    desiredSpeed = ROVER_SPEED_SLOW;
                    ticksRemaining = deg2tick(720);
                }
                else if (movingOriented == 1 && movementState == STATE_MOVING && orientBackwards == 1) {

                    desiredSpeed = ROVER_SPEED_STRAIGHT;
                    SetDirectionBackwards();
                    ticksRemaining = in2tick(50);


                }

                //Ignore tape for x seconds
                ignoreTapeCount++;
                if (ignoreTapeCount >= ignoreTapeMax) {
                    ignoringTape = 0;
                }

                //Handle PWM stuff
                m2PID = PID2(desiredSpeed, speed2);
            } else if (msgId == NAV_TIMER_COUNTER_5_ID_SENSOR) {
                //Motor 2 Encoder Message Handler
                speed1 = (receivemsgint & 0x0000ffff) - previousValue1;
                previousValue1 = receivemsgint & 0x0000ffff;

                //Handle remaining distance
                if (GetMotorDirection() != ROVER_DIRECTION_LEFT) {
                    HandleDistanceRemaining(&desiredSpeed, &ticksRemaining, speed1);

                    //Handle position and orientation
                    HandlePositionAndOrientation(speed1, GetMotorDirection(), CALCULATE_IN_CENTIMETERS);
                     
                    
                    mapSetYPosition(GetLocationY());
                    //Handle Daniel's server testing
                    //                    motorTestNavSendSpeed(speed1);
                }

                //Handle PWM stuff
                m1PID = PID1(desiredSpeed, speed1);
            } else if (msgId == NAV_COLOR_SENSOR_1_ID_SENSOR) {
                //Handle stuff from color sensor 1
                //Front left color sensor
                if (receivemsg[0] == COLOR_IS_BLUE) {
                    cs1OnTape = true;
                } else {
                    cs1OnTape = false;
                }
                HandleCornerColorSensor(NAV_COLOR_SENSOR_1_ID_SENSOR);
            } else if (msgId == NAV_COLOR_SENSOR_2_ID_SENSOR) {
                //Handle stuff from color sensor 2
                //Back left color sensor
                if (receivemsg[0] == COLOR_IS_BLUE) {
                    cs2OnTape = true;
                } else {
                    cs2OnTape = false;
                }
                HandleCornerColorSensor(NAV_COLOR_SENSOR_2_ID_SENSOR);
            } else if (msgId == NAV_COLOR_SENSOR_3_ID_SENSOR) {
                //Handle stuff from color sensor 3
                //Back side color sensor
                if (receivemsg[0] == COLOR_IS_BLUE) {
                    cs3OnTape = true;
                } else {
                    cs3OnTape = false;
                }
                HandleSideColorSensor(NAV_COLOR_SENSOR_3_ID_SENSOR);
            } else if (msgId == NAV_COLOR_SENSOR_4_ID_SENSOR) {
                //Handle stuff from color sensor 4
                //Front side color sensor
                if (receivemsg[0] == COLOR_IS_BLUE) {
                    cs4OnTape = true;
                } else {
                    cs4OnTape = false;
                }
                HandleSideColorSensor(NAV_COLOR_SENSOR_4_ID_SENSOR);
            } else if (msgId == NAV_MAPPING_ID_SENSOR) {
                //Handle stuff from the mapping queue
            } else if (msgId == NAV_PWM_TIMER_ID) {
                //Handle PWM timer messages
                Motor1SetPWM(GetPWMFromValue(m1PID, pwmCount));
                Motor2SetPWM(GetPWMFromValue(m2PID, pwmCount));
                pwmCount++;
                if (pwmCount >= 25) {
                    pwmCount = 0;
                }

                //Handle color sensor communication state machine
                i2cCount++;
                if (i2cCount >= 50) {
                    int currentState2 = DRV_TCS_HandleColorSensor(i2c1_handle, COLOR_SENSOR_ID_1);
                    i2cCount = 0;
                    Nop();
                } else if (i2cCount == 25) {
                    int currentState1 = DRV_TCS_HandleColorSensor(i2c2_handle, COLOR_SENSOR_ID_2);
                } else if (i2cCount == 10) {
                    int currentState3 = DRV_TCS_HandleColorSensor(i2c3_handle, COLOR_SENSOR_ID_3);
                } else if (i2cCount == 38) {
                    int currentState4 = DRV_TCS_HandleColorSensor(i2c4_handle, COLOR_SENSOR_ID_4);
                }
                Nop();
            } else if (msgId == NAV_OTHER_ID) {

                unsigned char pixyVal[RECEIVE_BUFFER_SIZE];

                //                sprintf(pixyVal, "*{\"S\":\"s\",\"T\":\"v\",\"M\":\"s\",\"N\":0,\"D\":[%d,%d],\"C\":123}~\n\r", 0, 1);
                //                commSendMsgToSendQueue(pixyVal);

                if (oriented == 0) {
                    ResetMovementQueue();
                    StopMovement();
                    AddMovement(in2tick(50), ROVER_DIRECTION_FORWARDS);
                    SetMovementGoal();
                    ignoringTape = 1;
                    ignoreTapeCount = 20;
                    oriented = 1;
                    movementState = STATE_ORIENTING;
                    flagAngleFromStart = GetOrientation();
                }
                //                unsigned char pixyVal[RECEIVE_BUFFER_SIZE];
                //                
                //                sprintf(pixyVal, "*{\"S\":\"s\",\"T\":\"v\",\"M\":\"s\",\"N\":0,\"D\":[%d,%d],\"C\":123}~\n\r", 0, 1);
                //                commSendMsgToSendQueue(pixyVal);
                //Handle a message from another source
                //Handle Daniel's server testing
                //                motorTestNavReceive(receivemsg, &desiredSpeed, &ticksRemaining);
                Nop();
            }
            //        }
        }
    }
}



/*******************************************************************************
 End of File
 */
