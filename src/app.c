/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

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

#include "app.h"
#include "app_public.h"

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

APP_DATA appData;

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

int app1Iter;
static QueueHandle_t gpioQueue;

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    Nop();
    
    gpioQueue = xQueueCreate(10, sizeof(unsigned int));
    if(gpioQueue == 0){
        dbgPauseAll();
    }
    
   
    
    
    //bool thing = DRV_TMR0_Start();
    //DRV_USART0_Open();
    
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

void app1SendMsgFromISR(unsigned int msg){
    dbgOutputLoc(DBG_LOC_APP1_TMR_ISR_ADD_MSG_TO_QUEUE_ENTER);
    BaseType_t xHigherPriorityTaskWoken =  pdTRUE;//pdFALSE;
   //xQueueSendFromISR(gpioQueue, (void *) &msg, &xHigherPriorityTaskWoken);//&xHigherPriorityTaskWoken);
     //xQueueSendFromISR(gpioQueue, &msg, &xHigherPriorityTaskWoken);
    //xQueueSendFromISR(gpioQueue, &msg, NULL);
    xQueueSendToBackFromISR(gpioQueue, &msg, NULL);
    dbgOutputLoc(DBG_LOC_APP1_TMR_ISR_ADD_MSG_TO_QUEUE_EXIT);
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    dbgOutputLoc(DBG_LOC_APP1_ENTER);

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
                
                unsigned int receivemsg;
                unsigned int testq = 1;
                app1Iter = 0;
                
                
                
                bool thing = DRV_TMR0_Start();
                bool motor = DRV_OC0_Start();
                bool mt2 = DRV_OC1_Start();
               
                dbgOutputLoc(DBG_LOC_APP1_BEFORE_WHILE);
                while(1){
                    //Nop();
                    dbgOutputLoc(DBG_LOC_APP1_BEFORE_RECEIVE);
                    BaseType_t receiveCheck = xQueueReceive(gpioQueue, &receivemsg, portMAX_DELAY);
                    dbgOutputLoc(DBG_LOC_APP1_AFTER_RECEIVE);
//                    xQueueReceive(gpioQueue, &receivemsg, portMAX_DELAY);
                    //if(xQueueReceive(gpioQueue, &receivemsg, portMAX_DELAY) == pdTRUE){
                    
                    if(receiveCheck == pdTRUE){
                        //Nop();
                        if (receivemsg == 1){
                            //Nop();
                            if (app1Iter == 0){
                                dbgOutputVal(0x54);
                                dbgUARTVal(0x54);
                                //dbgOutputLoc(0x54);
                                app1Iter++;
                            } else if (app1Iter == 1){
                                dbgOutputVal(0x65);
                                dbgUARTVal(0x65);
                                //dbgOutputLoc(0x65);
                                app1Iter++;
                            } else if (app1Iter == 2){
                                dbgOutputVal(0x61);
                                dbgUARTVal(0x61);
                                //dbgOutputLoc(0x61);
                                app1Iter++;
                            } else if (app1Iter == 3){
                                dbgOutputVal(0x6D);
                                dbgUARTVal(0x6D);
                                //dbgOutputLoc(0x6D);
                                app1Iter++;
                            } else if (app1Iter == 4){
                                dbgOutputVal(0x20);
                                dbgUARTVal(0x20);
                                //dbgOutputLoc(0x20);
                                app1Iter++;
                            } else {
                                dbgOutputVal(0x37);
                                dbgUARTVal(0x37);
                                //dbgOutputLoc(0x37);
                                app1Iter = 0;
                            }
                        }
                    }else{
                        Nop();
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
