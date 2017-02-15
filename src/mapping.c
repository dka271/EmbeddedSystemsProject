/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    mapping.c

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

#include "mapping.h"
#include "mapping_public.h"

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

MAPPING_DATA mappingData;

static QueueHandle_t mapQueue;

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
    void MAPPING_Initialize ( void )

  Remarks:
    See prototype in mapping.h.
 */

void MAPPING_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    mappingData.state = MAPPING_STATE_INIT;
    
    //Initialize the mapping queue
    mapQueue = xQueueCreate(10, sizeof(unsigned char[MAP_QUEUE_BUFFER_SIZE]));
    if(mapQueue == 0){
        dbgPauseAll();
    }

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

void mapSendMsgFromISR(unsigned char msg[MAP_QUEUE_BUFFER_SIZE]){
    BaseType_t xHigherPriorityTaskWoken =  pdTRUE;//pdFALSE;
    xQueueSendToBackFromISR(mapQueue, msg, NULL);
}

void mapSendMsg(unsigned char msg[MAP_QUEUE_BUFFER_SIZE]){
    BaseType_t xHigherPriorityTaskWoken =  pdTRUE;//pdFALSE;
    xQueueSendToBack(mapQueue, msg, portMAX_DELAY);
}
    
unsigned char mapCalculateChecksum(unsigned char msg[MAP_QUEUE_BUFFER_SIZE]){
    unsigned char sum = 0;
    unsigned int i;
    for (i = 0; i < MAP_QUEUE_BUFFER_SIZE - 1; i++){
        sum += msg[i];
    }
    return sum;
}


/******************************************************************************
  Function:
    void MAPPING_Tasks ( void )

  Remarks:
    See prototype in mapping.h.
 */

void MAPPING_Tasks ( void )
{
    dbgOutputLoc(DBG_LOC_MAP_ENTER);

    /* Check the application's current state. */
    switch ( mappingData.state )
    {
        /* Application's initial state. */
        case MAPPING_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                mappingData.state = MAPPING_STATE_SERVICE_TASKS;
            }
            break;
        }

        case MAPPING_STATE_SERVICE_TASKS:
        {
            unsigned char receivemsg[MAP_QUEUE_BUFFER_SIZE];
            
            dbgOutputLoc(DBG_LOC_MAP_BEFORE_WHILE);
            while(1){
                //Block until a message is received
                dbgOutputLoc(DBG_LOC_MAP_BEFORE_RECEIVE);
                BaseType_t receiveCheck = xQueueReceive(mapQueue, receivemsg, portMAX_DELAY);
                dbgOutputLoc(DBG_LOC_MAP_AFTER_RECEIVE);
                
                //Handle the message
                if(receiveCheck == pdTRUE){
                    //Convert the message into integer format
                    unsigned int receivemsgint0 = receivemsg[0] | (receivemsg[1] << 8) | (receivemsg[2] << 16) | (receivemsg[3] << 24);
                    unsigned int receivemsgint1 = receivemsg[4] | (receivemsg[5] << 8) | (receivemsg[6] << 16) | (receivemsg[7] << 24);
                    unsigned int receivemsgint2 = receivemsg[8] | (receivemsg[9] << 8) | (receivemsg[10] << 16) | (receivemsg[11] << 24);
                    unsigned int receivemsgint3 = receivemsg[12] | (receivemsg[13] << 8) | (receivemsg[14] << 16);
                    //Get the message ID
                    int msgId = (receivemsg[MAP_SOURCE_ID_IDX] & MAP_SOURCE_ID_MASK) >> MAP_SOURCE_ID_OFFSET;
                    //Handle a specific message
                    if (msgId == MAP_NAVIGATION_ID){
                        //Handle a message from the navigation thread
                        dbgOutputVal(receivemsg[0]);
                        dbgOutputVal(receivemsg[1]);
                        dbgOutputVal(receivemsg[2]);
                        dbgOutputVal(receivemsg[3]);
                        dbgOutputVal(receivemsg[4]);
                        dbgOutputVal(receivemsg[5]);
                        dbgOutputVal(receivemsg[6]);
                        dbgOutputVal(receivemsg[7]);
                        dbgOutputVal(receivemsg[8]);
                        dbgOutputVal(receivemsg[9]);
                        dbgOutputVal(receivemsg[10]);
                        dbgOutputVal(receivemsg[11]);
                        dbgOutputVal(receivemsg[12]);
                        dbgOutputVal(receivemsg[13]);
                        
                        unsigned char msg2[6];
                        msg2[0] = 0x1f;
                        msg2[1] = 0x2e;
                        msg2[2] = 0x3d;
                        msg2[3] = 0x4c;
                        msg2[4] = 0x5b;
                        msg2[COMM_SOURCE_ID_IDX] = (COMM_MAPPING_ID & 0x00000001) << COMM_SOURCE_ID_OFFSET;
                        msg2[COMM_CHECKSUM_IDX] = commCalculateChecksum(msg2);
                        commSendMsg(msg2);
                    }else if (msgId == MAP_PIXY_CAM_ID){
                        //Handle input from the pixy cam
                    }else if (msgId == MAP_ULTRASONIC_ID){
                        //Handle input from the ultrasonic sensor
                    }else if (msgId == MAP_IR_1_ID){
                        //Handle input from the first IR sensor
                    }else if (msgId == MAP_IR_2_ID){
                        //Handle input from the second IR sensor
                    }else if (msgId == MAP_MAPPING_TIMER_ID){
                        //Start sampling
                        dbgOutputVal(receivemsg[13]);
                        
                        unsigned char msg1[NAV_QUEUE_BUFFER_SIZE];
                        msg1[0] = 0x5a;
                        msg1[1] = 0x6c;
                        msg1[NAV_SOURCE_ID_IDX] = (NAV_MAPPING_ID_SENSOR & 0x00000007) << NAV_SOURCE_ID_OFFSET;
                        msg1[NAV_CHECKSUM_IDX] = navCalculateChecksum(msg1);
                        navSendMsg(msg1);
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
