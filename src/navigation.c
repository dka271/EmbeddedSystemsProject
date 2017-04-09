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

NAVIGATION_DATA navigationData;

static QueueHandle_t navQueue;

void NAVIGATION_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    navigationData.state = NAVIGATION_STATE_INIT;
    
    //Initialize the navigation queue
    navQueue = xQueueCreate(10, sizeof(unsigned char[NAV_QUEUE_BUFFER_SIZE]));
    if(navQueue == 0){
        dbgPauseAll();
    }
}

void navSendMsgFromISR(unsigned char msg[NAV_QUEUE_BUFFER_SIZE]){
    BaseType_t xHigherPriorityTaskWoken =  pdTRUE;//pdFALSE;
    xQueueSendToBackFromISR(navQueue, msg, NULL);
}

void navSendMsg(unsigned char msg[NAV_QUEUE_BUFFER_SIZE]){
    BaseType_t xHigherPriorityTaskWoken =  pdTRUE;//pdFALSE;
    xQueueSendToBack(navQueue, msg, portMAX_DELAY);
}
    
unsigned char navCalculateChecksum(unsigned char msg[NAV_QUEUE_BUFFER_SIZE]){
    unsigned char sum = 0;
    unsigned int i;
    for (i = 0; i < NAV_QUEUE_BUFFER_SIZE - 1; i++){
        sum += msg[i];
    }
    return sum;
}


/******************************************************************************
  Function:
    void NAVIGATION_Tasks ( void )

  Remarks:
    See prototype in navigation.h.
 */

void NAVIGATION_Tasks ( void )
{
    dbgOutputLoc(DBG_LOC_NAV_ENTER);
    //Variables to be used for navigation
    unsigned char receivemsg[NAV_QUEUE_BUFFER_SIZE];
    unsigned int previousValue1 = 0;
    unsigned int speed1;
    unsigned int previousValue2 = 0;
    unsigned int speed2;
    unsigned int pwmCount = 0;
    unsigned int desiredSpeed = ROVER_SPEED_STOPPED;
    int ticksRemaining = ROVER_TICKS_REMAINING_NONE;
    int m1PID;
    int m2PID;
//    unsigned char oriented = 0;
        unsigned char oriented = 0;
//    SetDirectionForwards();
    SetDirectionClockwise();

    dbgOutputLoc(DBG_LOC_NAV_BEFORE_WHILE);
    
    
    //I2C Initialization Stuff
    //Open the I2C
    int i2cCount = -100;//Set this low so the color sensors are guaranteed to receive power by the time we start initializing them
    while (DRV_I2C_Status(sysObj.drvI2C0) != SYS_STATUS_READY){
        //Wait for the I2C to be ready to be opened
        //FOR TESTING
        if (COLOR_SENSOR_SERVER_TESTING){
//            sprintf(testMsg, "Waiting for I2C 1...");
//            commSendMsgToWifiQueue(testMsg);
        }
        //END FOR TESTING
    }
    DRV_HANDLE i2c1_handle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE);
    while (DRV_I2C_Status(sysObj.drvI2C1) != SYS_STATUS_READY){
        //Wait for the I2C to be ready to be opened
        //FOR TESTING
        if (COLOR_SENSOR_SERVER_TESTING){
//            sprintf(testMsg, "Waiting for I2C 2...");
//            commSendMsgToWifiQueue(testMsg);
        }
        //END FOR TESTING
    }
    DRV_HANDLE i2c2_handle = DRV_I2C_Open(DRV_I2C_INDEX_1, DRV_IO_INTENT_READWRITE);
    
    
    
    while (DRV_I2C_Status(sysObj.drvI2C2) != SYS_STATUS_READY){
        //Wait for the I2C to be ready to be opened
        //FOR TESTING
        if (COLOR_SENSOR_SERVER_TESTING){
//            sprintf(testMsg, "Waiting for I2C 2...");
//            commSendMsgToWifiQueue(testMsg);
        }
        //END FOR TESTING
    }
    DRV_HANDLE i2c3_handle = DRV_I2C_Open(DRV_I2C_INDEX_2, DRV_IO_INTENT_READWRITE);
    
    //Init the I2C state machine
    DRV_TCS_HandleColorSensor(NULL, COLOR_SENSOR_RESET_STATE_MACHINE);
    
    
    while(1){
        //Block until a message is received
        dbgOutputLoc(DBG_LOC_NAV_BEFORE_RECEIVE);
        BaseType_t receiveCheck = xQueueReceive(navQueue, receivemsg, portMAX_DELAY);
        dbgOutputLoc(DBG_LOC_NAV_AFTER_RECEIVE);

        //Handle the message
        if(receiveCheck == pdTRUE){
            //Convert the message into integer format
            unsigned int receivemsgint = receivemsg[0] | (receivemsg[1] << 8) | (receivemsg[2] << 16) | (receivemsg[3] << 24);
            //Get the message ID
            int msgId = (receivemsg[NAV_SOURCE_ID_IDX] & NAV_SOURCE_ID_MASK) >> NAV_SOURCE_ID_OFFSET;
            //Handle a specific message
 
            if (msgId == NAV_TIMER_COUNTER_3_ID_SENSOR){
                //Motor 2 Encoder Message Handler
                speed2 = (receivemsgint & 0x0000ffff) - previousValue2;
                previousValue2 = receivemsgint & 0x0000ffff;
                
                //Handle remaining distance
                if (GetMotorDirection() == ROVER_DIRECTION_LEFT){
                    HandleDistanceRemaining(&desiredSpeed, &ticksRemaining, speed2);
                
                    //Handle Daniel's server testing
                    motorTestNavSendSpeed(speed2);
                }
                
                if(oriented == 0){
                    desiredSpeed = ROVER_SPEED_SLOW;
                    ticksRemaining = deg2tick(720);
                }

                //Handle PWM stuff
                m2PID = PID2(desiredSpeed, speed2);
                Nop();
            }else if (msgId == NAV_TIMER_COUNTER_5_ID_SENSOR){
                //Motor 2 Encoder Message Handler
                speed1 = (receivemsgint & 0x0000ffff) - previousValue1;
                previousValue1 = receivemsgint & 0x0000ffff;
                
                //Handle remaining distance
                if (GetMotorDirection() != ROVER_DIRECTION_LEFT){
                    HandleDistanceRemaining(&desiredSpeed, &ticksRemaining, speed1);
                
                    //Handle Daniel's server testing
                    motorTestNavSendSpeed(speed1);
                }

                //Handle PWM stuff
                m1PID = PID1(desiredSpeed, speed1);
                
                //Handle unit testing
//                if (UNIT_TESTING){
//                    encoderSpeedTest(speed1);
//                }
                Nop();
            }else if (msgId == NAV_COLOR_SENSOR_1_ID_SENSOR){
                //Handle stuff from color sensor 1
            }else if (msgId == NAV_COLOR_SENSOR_2_ID_SENSOR){
                //Handle stuff from color sensor 2
            }else if (msgId == NAV_COLOR_SENSOR_3_ID_SENSOR){
                //Handle stuff from color sensor 3
                if (UNIT_TESTING){
                    navQueueReceiveTest(receivemsg);
                }
            }else if (msgId == NAV_MAPPING_ID_SENSOR){
                //Handle stuff from the mapping queue
            }else if (msgId == NAV_PWM_TIMER_ID){
                //Handle PWM timer messages
                Motor1SetPWM(GetPWMFromValue(m1PID, pwmCount));
                Motor2SetPWM(GetPWMFromValue(m2PID, pwmCount));
                pwmCount++;
                if (pwmCount >= 25){
                    pwmCount = 0;
                }
                
                //Handle color sensor communication state machine
                i2cCount++;
                if (i2cCount >= 50){
                    int currentState2 = DRV_TCS_HandleColorSensor(i2c1_handle, COLOR_SENSOR_ID_2);
                    i2cCount = 0;
                    Nop();
                }else if (i2cCount == 25){
                    int currentState1 = DRV_TCS_HandleColorSensor(i2c2_handle, COLOR_SENSOR_ID_1);
                }
                else if (i2cCount == 10){
                    int currentState1 = DRV_TCS_HandleColorSensor(i2c3_handle, COLOR_SENSOR_ID_3);
                }
                Nop();
            }else if (msgId == NAV_OTHER_ID){
                
                unsigned char pixyVal[RECEIVE_BUFFER_SIZE];
                
//                sprintf(pixyVal, "*{\"S\":\"s\",\"T\":\"v\",\"M\":\"s\",\"N\":0,\"D\":[%d,%d],\"C\":123}~\n\r", 0, 1);
//                commSendMsgToSendQueue(pixyVal);
                
                if (oriented == 0){
//                    desiredSpeed = ROVER_SPEED_STOPPED;
                    SetDirectionForwards();
                    desiredSpeed = ROVER_SPEED_STRAIGHT;
                    ticksRemaining = deg2tick(100);
//                    ticksRemaining = 0;
                    oriented = 1;
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
