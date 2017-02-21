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


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "navigation.h"
#include "navigation_public.h"
#include "ms2test.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

NAVIGATION_DATA navigationData;

static QueueHandle_t navQueue;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void NAVIGATION_Initialize ( void )

  Remarks:
    See prototype in navigation.h.
 */

void NAVIGATION_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    navigationData.state = NAVIGATION_STATE_INIT;
    
    //Initialize the navigation queue
    navQueue = xQueueCreate(10, sizeof(unsigned char[NAV_QUEUE_BUFFER_SIZE]));
    if(navQueue == 0){
        dbgPauseAll();
    }

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
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

int GetPWMFromValue(unsigned int val, unsigned int count){
    if (val > 25){
        val = 25;
    }
    int pwm = (count % 25) < val;
    return pwm;
}


int Kp = 0;//Test and calibrate this number//1
int Ti = 6;//Try to eliminate past errors in Ti*dt
int Td = 8;//Try to predict errors Td*dt in the future
//Set gain parameters
//int Ki = (int) (Kp / Ti);
//int Kd = Kp * Td;
int Ki = 0;//3
int Kd = 0;//80
int dt = 4;//4 ms
int PIDOffsetDiv = 13;//9
int PIDDiv = 1;//1000
int errorDiv = 1;
int PID1_PreviousError = 0;
int PID1_I = 0;
int PID1_Out = 0;
int PID1(unsigned int setpoint, unsigned int actual){
    int PIDOffset = (int) (setpoint/PIDOffsetDiv);
    int error = (int) ((setpoint - actual));
    /*
    PID1_I += error*dt;
    
    //Limit the Integral component
    if (PID1_I > 1000000){
        PID1_I = 1000000;
    }else if (PID1_I < -1000000){
        PID1_I = -1000000;
    }
    
    int PID1_D = (int) ((error - PID1_PreviousError)/dt);
    int out = (Kp*error*dt) + (Kd*PID1_D) + (Ki*PID1_I);
    PID1_PreviousError = error;
    //Limit the output to [-10,10]
    out = (int) out / PIDDiv;
    //out += PIDOffset;
    */
    if (error > 0){
        PID1_Out++;
    }else{
        PID1_Out--;
    }
    if (PID1_Out > 25){
        PID1_Out = 25;
    }else if (PID1_Out < 0){
        PID1_Out = 0;
    }
    return PID1_Out;
}

int PID2_PreviousError = 0;
int PID2_I = 0;
int PID2_Out = 0;
int PID2(unsigned int setpoint, unsigned int actual){
    int PIDOffset = (int) (setpoint/PIDOffsetDiv);
    int error = (int) ((setpoint - actual));
    /*
    PID2_I += error*dt;
    
    //Limit the Integral component
    if (PID2_I > 1000000){
        PID2_I = 1000000;
    }else if (PID2_I < -1000000){
        PID2_I = -1000000;
    }
    
    int PID2_D = (int) ((error - PID2_PreviousError)/dt);
    int out = (Kp*error*dt) + (Kd*PID2_D) + (Ki*PID2_I);
    PID2_PreviousError = error;
    //Limit the output to [0,20]
    out = (int) out / PIDDiv;
    */
    //out += PIDOffset;
    if (error > 0){
        PID2_Out++;
    }else{
        PID2_Out--;
    }
    if (PID2_Out > 25){
        PID2_Out = 25;
    }else if (PID2_Out < 0){
        PID2_Out = 0;
    }
    return PID2_Out;
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

    /* Check the application's current state. */
    switch ( navigationData.state )
    {
        /* Application's initial state. */
        case NAVIGATION_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                navigationData.state = NAVIGATION_STATE_SERVICE_TASKS;
            }
            break;
        }

        case NAVIGATION_STATE_SERVICE_TASKS:
        {
            unsigned char receivemsg[NAV_QUEUE_BUFFER_SIZE];
            unsigned int previousValue1 = 0;
            unsigned int speed1;
            unsigned int previousValue2 = 0;
            unsigned int speed2;
            unsigned int pwmCount = 0;
            unsigned int desiredSpeed = 50;
            int m1PID;
            int m2PID;
            //Motor1SetPWM(1);
            //Motor2SetPWM(1);
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
                        
                        if (UNIT_TESTING){
                            encoderSpeedTest(speed1);
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
                        //int ticksTarget = TEST_SPEED_TICKS;
                        Motor1SetPWM(GetPWMFromValue(m1PID, pwmCount));
                        Motor2SetPWM(GetPWMFromValue(m2PID, pwmCount));
//                        dbgOutputVal(m1PID);
                        //dbgOutputVal(m2PID);
                        pwmCount++;
                        if (pwmCount >= 25){
                            pwmCount = 0;
                        }
                    }
                }
            }
        
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
