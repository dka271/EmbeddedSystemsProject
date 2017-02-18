/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    debug.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef DEBUG_H    /* Guard against multiple inclusion */
#define DEBUG_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */
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


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */

    /*  A brief description of a section can be given directly below the section
        banner.
     */


    /* ************************************************************************** */
    /** Descriptive Constant Name

      @Summary
        Brief one-line summary of the constant.
    
      @Description
        Full description, explaining the purpose and usage of the constant.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.
    
      @Remarks
        Any additional remarks
     */
#define EXAMPLE_CONSTANT 0
    
//Constants that refer to specific parts of the code
#define DBG_LOC_PAUSE_ALL 1
#define DBG_LOC_TMR0_ISR_ENTER 2
#define DBG_LOC_TMR0_ISR_EXIT 3
#define DBG_LOC_TMR0_ISR_BEFORE_SEND 4
#define DBG_LOC_TMR0_ISR_AFTER_SEND 5
#define DBG_LOC_APP1_ENTER 6
#define DBG_LOC_APP1_BEFORE_WHILE 7
#define DBG_LOC_APP1_BEFORE_SEND 8
#define DBG_LOC_APP1_BEFORE_RECEIVE 9
#define DBG_LOC_APP1_AFTER_RECEIVE 10
#define DBG_LOC_APP1_TMR_ISR_ADD_MSG_TO_QUEUE_ENTER 11
#define DBG_LOC_APP1_TMR_ISR_ADD_MSG_TO_QUEUE_EXIT 12
#define DBG_LOC_TMR1_ISR_ENTER 13
#define DBG_LOC_TMR1_ISR_EXIT 14
#define DBG_LOC_TMR2_ISR_ENTER 15
#define DBG_LOC_TMR2_ISR_EXIT 16
#define DBG_LOC_NAV_ENTER 26
#define DBG_LOC_NAV_BEFORE_WHILE 27
#define DBG_LOC_NAV_BEFORE_SEND 28
#define DBG_LOC_NAV_BEFORE_RECEIVE 29
#define DBG_LOC_NAV_AFTER_RECEIVE 30
#define DBG_LOC_MAP_ENTER 31
#define DBG_LOC_MAP_BEFORE_WHILE 32
#define DBG_LOC_MAP_BEFORE_SEND 33
#define DBG_LOC_MAP_BEFORE_RECEIVE 34
#define DBG_LOC_MAP_AFTER_RECEIVE 35
#define DBG_LOC_COMM_ENTER 36
#define DBG_LOC_COMM_BEFORE_WHILE 37
#define DBG_LOC_COMM_BEFORE_SEND 38
#define DBG_LOC_COMM_BEFORE_RECEIVE 39
#define DBG_LOC_COMM_AFTER_RECEIVE 40
#define DBG_LOC_COMM_BEFORE_USART_SEND 41
#define DBG_LOC_COMM_AFTER_USART_SEND 42
#define DBG_LOC_COMM_ENTER_WRITE 43
#define DBG_LOC_COMM_LEAVE_WRITE 44
#define DBG_LOC_COMM_ENTER_READ 45
#define DBG_LOC_COMM_LEAVE_READ 46
#define DBG_LOC_COMM_INSIDE_RX_INT 47
#define DBG_LOC_COMM_ENTER_UART_WRITE_ISR 48
#define DBG_LOC_COMM_LEAVE_UART_WRITE_ISR 49
#define DBG_LOC_COMM_ENTER_UART_ISR 50
#define DBG_LOC_COMM_WRITE_LOOP 51
#define DBG_LOC_COMM_DISABLE_UART_ISR 52
#define DBG_LOC_COMM_BEFORE_WHILE_NEW_VAL 88


    // *****************************************************************************
    // *****************************************************************************
    // Section: Data Types
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */


    // *****************************************************************************

    /** Descriptive Data Type Name

      @Summary
        Brief one-line summary of the data type.
    
      @Description
        Full description, explaining the purpose and usage of the data type.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Remarks
        Any additional remarks
        <p>
        Describe enumeration elements and structure and union members above each 
        element or member.
     */


    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

    //This function outputs an 8-bit value to 8 specific IO lines
    void dbgOutputVal(unsigned char outVal);
    
    //This function outputs an 8-bit value to the UART
    //It is only meant to be called once every 100 milliseconds 
    void dbgUARTVal(unsigned char outVal);
    
    //This function uses a string 
    void dbgUARTString(unsigned char* outVal, unsigned int numChars);
    
    //This function outputs an 8-bit value to 8 specific IO lines that are
    //different from dbgOutputVal
    //This function is to be called at various locations in the code
    void dbgOutputLoc(unsigned char outVal);
    
    //This function brings everything to a halt in an obvious way
    //This is only meant to be called when we receive an error
    //This will output a specific value using dbgOutputLoc
    void dbgPauseAll();
    
    /*  A brief description of a section can be given directly below the section
        banner.
     */

    // *****************************************************************************
    /**
      @Function
        int ExampleFunctionName ( int param1, int param2 ) 

      @Summary
        Brief one-line description of the function.

      @Description
        Full description, explaining the purpose and usage of the function.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Precondition
        List and describe any required preconditions. If there are no preconditions,
        enter "None."

      @Parameters
        @param param1 Describe the first parameter to the function.
    
        @param param2 Describe the second parameter to the function.

      @Returns
        List (if feasible) and describe the return values of the function.
        <ul>
          <li>1   Indicates an error occurred
          <li>0   Indicates an error did not occur
        </ul>

      @Remarks
        Describe any special behavior not described above.
        <p>
        Any additional remarks.

      @Example
        @code
        if(ExampleFunctionName(1, 2) == 0)
        {
            return 3;
        }
     */
    int ExampleFunction(int param1, int param2);


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* DEBUG_H */

/* *****************************************************************************
 End of File
 */
