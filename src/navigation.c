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
    
    //Variables to be used for testing
    int Is_Testing_Speed = 0;
    
    //Variables to be used for navigation
    unsigned char receivemsg[NAV_QUEUE_BUFFER_SIZE];
    unsigned int previousValue1 = 0;
    unsigned int speed1;
    unsigned int previousValue2 = 0;
    unsigned int speed2;
    unsigned int pwmCount = 0;
    unsigned int desiredSpeed = 0;
    int m1PID;
    int m2PID;
    Motor1SetPWM(0);
    Motor2SetPWM(0);
    Motor1SetDirection(MOTOR_1_FORWARDS);
    Motor2SetDirection(MOTOR_2_FORWARDS);

    dbgOutputLoc(DBG_LOC_NAV_BEFORE_WHILE);
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
                //dbgOutputVal((receivemsgint & 0x00ff0000) >> 16);
                //dbgOutputVal((receivemsgint & 0x0000ff00) >> 8);
                //dbgOutputVal(receivemsgint & 0x000000ff);
                speed2 = (receivemsgint & 0x0000ffff) - previousValue2;
                //dbgUARTVal(speed2);
                previousValue2 = receivemsgint & 0x0000ffff;

                //Handle PWM stuff
                m2PID = PID2(desiredSpeed, speed2);
            }else if (msgId == NAV_TIMER_COUNTER_5_ID_SENSOR){
                //Motor 2 Encoder Message Handler
                //dbgOutputVal((receivemsgint & 0x00ff0000) >> 16);
                //dbgOutputVal((receivemsgint & 0x0000ff00) >> 8);
                //dbgOutputVal(receivemsgint & 0x000000ff);
                speed1 = (receivemsgint & 0x0000ffff) - previousValue1;
//                        dbgUARTVal(speed1);
//                        dbgOutputVal(speed1);
                previousValue1 = receivemsgint & 0x0000ffff;

                //Handle PWM stuff
                m1PID = PID1(desiredSpeed, speed1);
                
                //Change the desired speed for testing, comment this out when done
//                desiredSpeed++;
//                if (desiredSpeed > 90){
//                    desiredSpeed = 0;
//                }

                if (UNIT_TESTING){
                    encoderSpeedTest(speed1);
                }
                if (Is_Testing_Speed){
                    //Handle server testing of the speed
                    char commMsg[COMM_QUEUE_BUFFER_SIZE];
                    commMsg[0] = speed1 & 0xff;
                    commMsg[1] = NAV_MOTOR_SPEED_ID;
                    commMsg[COMM_SOURCE_ID_IDX] = COMM_OTHER_ID;
                    commMsg[COMM_CHECKSUM_IDX] = commCalculateChecksum(commMsg);
                    commSendMsg(commMsg);
                }
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
                //dbgOutputVal((receivemsgint & 0x00ff0000) >> 16);
                //dbgOutputVal((receivemsgint & 0x0000ff00) >> 8);
                //dbgOutputVal(receivemsgint & 0x000000ff);

                unsigned char msg[MAP_QUEUE_BUFFER_SIZE];
                msg[0] = 0;
                msg[1] = 0x11;
                msg[2] = 0x22;
                msg[3] = 0x33;
                msg[4] = 0x44;
                msg[5] = 0x55;
                msg[6] = 0x66;
                msg[7] = 0x77;
                msg[8] = 0x88;
                msg[9] = 0x99;
                msg[10] = 0xaa;
                msg[11] = 0xbb;
                msg[12] = 0xcc;
                msg[MAP_SOURCE_ID_IDX] = (MAP_NAVIGATION_ID & 0x00000007) << MAP_SOURCE_ID_OFFSET;
                msg[MAP_CHECKSUM_IDX] = mapCalculateChecksum(msg);
                mapSendMsg(msg);
            }else if (msgId == NAV_PWM_TIMER_ID){
                //Handle PWM timer messages
                //int ticksTarget = TEST_SPEED_TICKS;
                Motor1SetPWM(GetPWMFromValue(m1PID, pwmCount));
                Motor2SetPWM(GetPWMFromValue(m2PID, pwmCount));
//                        dbgOutputVal(m1PID);
                //dbgOutputVal(m2PID);
                pwmCount++;
                if (pwmCount >= 25){
                    pwmCount = 0;
                }
            }else if (msgId == NAV_OTHER_ID){
                //Handle a message from another source
//                Nop();
                if (receivemsg[1] == NAV_MOTOR_SPEED_ID){
                    //Got speed from the test server
                    desiredSpeed = (int) receivemsg[0];
                    dbgOutputVal(desiredSpeed);
                    if (desiredSpeed == 0){
                        Is_Testing_Speed = 0;
                    }else{
                        Is_Testing_Speed = 1;
                    }
                }
            }
        }
    }
}

 

/*******************************************************************************
 End of File
 */
