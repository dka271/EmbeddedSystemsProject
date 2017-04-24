/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include <xc.h>
#include <math.h>
#include <sys/attribs.h>
#include "app_public.h"
#include "navigation_public.h"
#include "mapping_public.h"
#include "communication_public.h"
#include "system_definitions.h"

//Declare privates
#define ADC_NUM_SAMPLE_PER_AVERAGE 16
#define PIXY_CENTER_VALUE 512
#define PIXY_THRESHOLD_VALUE 20
#define ADC_DELAY 25

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
int incr = 0;
bool flagDetected = false;

void IntHandlerDrvAdc(void) {
    /* Clear ADC Interrupt Flag */

    ADC_SAMPLE irSensor1 = 0;
    ADC_SAMPLE sonarSensor0 = 0;
    ADC_SAMPLE pixy = 0;
    uint8_t i = 0;

    for (i = 0; i < 13; i += 3) {
        irSensor1 += PLIB_ADC_ResultGetByIndex(ADC_ID_1, i);
        sonarSensor0 += PLIB_ADC_ResultGetByIndex(ADC_ID_1, i + 1);
        pixy += PLIB_ADC_ResultGetByIndex(ADC_ID_1, i + 2);
    }
    irSensor1 += PLIB_ADC_ResultGetByIndex(ADC_ID_1, i);

    irSensor1 = irSensor1 / 6;
    sonarSensor0 = sonarSensor0 / 5;
   
    if(!flagDetected){
    pixy = pixy / 5;
    }




    unsigned char sendIR1[MAP_QUEUE_BUFFER_SIZE];
    unsigned char sendUltra0[MAP_QUEUE_BUFFER_SIZE];
    unsigned char sendPixy[NAV_QUEUE_BUFFER_SIZE];





    sendIR1[0] = (irSensor1 & 0xFF00) >> 8;
    sendIR1[1] = (irSensor1 & 0x00FF);

    sendUltra0[0] = (sonarSensor0 & 0xFF00) >> 8;
    sendUltra0[1] = (sonarSensor0 & 0x00FF);

    
     sendIR1[MAP_SOURCE_ID_IDX] = (MAP_IR_2_ID & (MAP_SOURCE_ID_MASK >> MAP_SOURCE_ID_OFFSET)) << MAP_SOURCE_ID_OFFSET;
    sendUltra0[MAP_SOURCE_ID_IDX] = (MAP_ULTRASONIC_ID & (MAP_SOURCE_ID_MASK >> MAP_SOURCE_ID_OFFSET)) << MAP_SOURCE_ID_OFFSET;
    
    if(!flagDetected){
    sendPixy[0] = (pixy & 0xFF00) >> 8;
    sendPixy[1] = (pixy & 0x00FF);
    }

   
    if(!flagDetected){
    sendPixy[NAV_SOURCE_ID_IDX] = (NAV_OTHER_ID & 0x00000007) << NAV_SOURCE_ID_OFFSET;
    }
    //commented out
    if (incr == ADC_DELAY) {

        
//        mapSendMsgFromISR(sendUltra0);
//        mapSendMsgFromISR(sendIR1);
        
        if(!flagDetected){
            if(abs(pixy - PIXY_CENTER_VALUE) <= PIXY_THRESHOLD_VALUE){
            navSendMsgFromISR(sendPixy);
            flagDetected = true;
            }
        }
        incr = 0;
    } else {
        incr++;
    }
    
    //commented out




    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1);
    PLIB_ADC_SampleAutoStartEnable(ADC_ID_1);
}

void IntHandlerDrvTmrInstance0(void) {
    dbgOutputLoc(DBG_LOC_TMR0_ISR_ENTER);

    //Sample the timer counters and send their values to the navigation queue
    unsigned short t3 = TMR3;
    unsigned short t5 = TMR5;
    unsigned char msg2[NAV_QUEUE_BUFFER_SIZE];
    dbgOutputLoc(DBG_LOC_TMR0_ISR_BEFORE_SEND);
    msg2[0] = t3 & 0x00ff;
    msg2[1] = (t3 & 0xff00) >> 8;
    msg2[NAV_SOURCE_ID_IDX] = (NAV_TIMER_COUNTER_3_ID_SENSOR & 0x00000007) << NAV_SOURCE_ID_OFFSET;
    msg2[NAV_CHECKSUM_IDX] = navCalculateChecksum(msg2);
    navSendMsgFromISR(msg2);
    msg2[0] = t5 & 0x00ff;
    msg2[1] = (t5 & 0xff00) >> 8;
    msg2[NAV_SOURCE_ID_IDX] = (NAV_TIMER_COUNTER_5_ID_SENSOR & 0x00000007) << NAV_SOURCE_ID_OFFSET;
    msg2[NAV_CHECKSUM_IDX] = navCalculateChecksum(msg2);
    navSendMsgFromISR(msg2);

    unsigned char msg3[MAP_QUEUE_BUFFER_SIZE];
    msg3[MAP_SOURCE_ID_IDX] = (MAP_MAPPING_TIMER_ID & 0x00000007) << MAP_SOURCE_ID_OFFSET;
    msg3[MAP_CHECKSUM_IDX] = mapCalculateChecksum(msg3);
    mapSendMsgFromISR(msg3);
    dbgOutputLoc(DBG_LOC_TMR0_ISR_AFTER_SEND);

    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_TIMER_2);
    unsigned char msg[COMM_QUEUE_BUFFER_SIZE];
    msg[COMM_SOURCE_ID_IDX] = (COMM_SEND_ID & 0x00000003) << COMM_SOURCE_ID_OFFSET;
    msg[COMM_CHECKSUM_IDX] = commCalculateChecksum(msg);
    commSendMsgFromISR(msg);
    dbgOutputLoc(DBG_LOC_TMR0_ISR_EXIT);
}

void IntHandlerDrvTmrInstance1(void) {
    dbgOutputLoc(DBG_LOC_TMR1_ISR_ENTER);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_TIMER_3);
    dbgOutputLoc(DBG_LOC_TMR1_ISR_EXIT);
}

void IntHandlerDrvTmrInstance2(void) {
    dbgOutputLoc(DBG_LOC_TMR2_ISR_ENTER);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_TIMER_5);
    dbgOutputLoc(DBG_LOC_TMR2_ISR_EXIT);
}

void IntHandlerDrvUsartInstance0(void) {
    dbgOutputLoc(DBG_LOC_COMM_ENTER_UART_ISR);
    if (SYS_INT_SourceStatusGet(INT_SOURCE_USART_1_RECEIVE)) {
        if (PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)) {
            char bufferToWriteTo[COMM_QUEUE_BUFFER_SIZE];
            //int MY_BUFFER_SIZE = 7;
            readPublic2(bufferToWriteTo, COMM_QUEUE_BUFFER_SIZE - 2);
            //            if (readPublicIntoBuffer2(bufferToWriteTo)){
            //                bufferToWriteTo[COMM_SOURCE_ID_IDX] = (COMM_UART_ID & 0x00000003) << COMM_SOURCE_ID_OFFSET;
            //                bufferToWriteTo[COMM_CHECKSUM_IDX] = commCalculateChecksum(bufferToWriteTo);
            //                commSendMsgFromISR(bufferToWriteTo);
            //            }
            //readPublicIntoBuffer();

            dbgOutputLoc(DBG_LOC_COMM_INSIDE_RX_INT);
        }
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
        //SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_RECEIVE);
    }
    if (SYS_INT_SourceStatusGet(INT_SOURCE_USART_1_TRANSMIT)) {
        dbgOutputLoc(DBG_LOC_COMM_ENTER_UART_WRITE_ISR);
        while (1) {
            if (!checkIfSendQueueIsEmpty()) {
                //                unsigned char bufferToReadFrom[SEND_QUEUE_BUFFER_SIZE];
                unsigned char bufferToReadFrom[7];
                if (PLIB_USART_TransmitterBufferIsFull(USART_ID_1)) {
                    break;
                }
                uartReceiveFromSendQueueInISR(bufferToReadFrom);
                writePublic(bufferToReadFrom);
            } else {
                dbgOutputLoc(DBG_LOC_COMM_DISABLE_UART_ISR);
                PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
                break;

            }
        }

        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    }

    if (SYS_INT_SourceStatusGet(INT_SOURCE_USART_1_ERROR)) {
        dbgPauseAll();
    }
    dbgOutputLoc(DBG_LOC_COMM_LEAVE_UART_WRITE_ISR);
}

void IntHandlerDrvTmrInstance3(void) {
    /*unsigned short t3 = TMR3;
    unsigned short t5 = TMR5;
    unsigned char msg1[NAV_QUEUE_BUFFER_SIZE];
    dbgOutputLoc(DBG_LOC_TMR0_ISR_BEFORE_SEND);
    msg1[0] = t3 & 0x00ff;
    msg1[1] = (t3 & 0xff00) >> 8;
    msg1[NAV_SOURCE_ID_IDX] = (NAV_TIMER_COUNTER_3_ID_SENSOR & 0x00000007) << NAV_SOURCE_ID_OFFSET;
    msg1[NAV_CHECKSUM_IDX] = navCalculateChecksum(msg1);
    navSendMsgFromISR(msg1);
    unsigned char msg2[NAV_QUEUE_BUFFER_SIZE];
    msg2[0] = t5 & 0x00ff;
    msg2[1] = (t5 & 0xff00) >> 8;
    msg2[NAV_SOURCE_ID_IDX] = (NAV_TIMER_COUNTER_5_ID_SENSOR & 0x00000007) << NAV_SOURCE_ID_OFFSET;
    msg2[NAV_CHECKSUM_IDX] = navCalculateChecksum(msg2);
    navSendMsgFromISR(msg2);*/
    unsigned char msg3[NAV_QUEUE_BUFFER_SIZE];
    msg3[0] = 0;
    msg3[1] = 0;
    msg3[NAV_SOURCE_ID_IDX] = (NAV_PWM_TIMER_ID & 0x00000007) << NAV_SOURCE_ID_OFFSET;
    msg3[NAV_CHECKSUM_IDX] = navCalculateChecksum(msg3);
    navSendMsgFromISR(msg3);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_TIMER_4);
}


void IntHandlerDrvI2CInstance0(void)
{
    DRV_I2C_Tasks(sysObj.drvI2C0);
 
}
         
 
   
  
void IntHandlerDrvI2CInstance1(void) 
{
    DRV_I2C_Tasks(sysObj.drvI2C1);
 
}
         
 
     
void IntHandlerDrvI2CInstance2(void) 
{
    DRV_I2C_Tasks(sysObj.drvI2C2);
 
}       
 
     
void IntHandlerDrvI2CInstance3(void) 
{
    DRV_I2C_Tasks(sysObj.drvI2C3);
 
}


/*******************************************************************************
 End of File
 */