/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    mapping.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _MAPPING_H
#define _MAPPING_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include "system_config.h"
#include "system_definitions.h"
#include "system/system.h"
#include "system/clk/sys_clk.h"
#include "driver/usart/drv_usart.h"
#include "system/devcon/sys_devcon.h"
#include <sys/appio.h>
#include <GenericTypeDefs.h>
#include "system/common/sys_module.h"
#include "system/msg/sys_msg.h"
#include "debug.h"
#include "motor.h"
#include "navigation_public.h"
#include "communication_public.h"
#include "mapping_public.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
    
//Global defines
#define MAX_NUMBER_OF_OBJECTS 32
#define MAX_NUMBER_OF_DELETED_OBJECTS 10
#define NUMBER_OF_ROVERS 8
    
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	MAPPING_STATE_INIT=0,
	MAPPING_STATE_SERVICE_TASKS,

	/* TODO: Define states used by the application state machine. */

} MAPPING_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    MAPPING_STATES state;

    /* TODO: Define any additional data used by the application. */
//    short OCCUPANCY_GRID[MAP_OCCUPANCY_ROWS][MAP_OCCUPANCY_COLUMNS];
// maybe change to a 1D array
    short OCCUPANCY_GRID[MAP_OCCUPANCY_ROWS][MAP_OCCUPANCY_COLUMNS];
//    unsigned char FLOOD_FILL_PAINT[MAP_OCCUPANCY_ROWS][MAP_OCCUPANCY_COLUMNS];
    short rover_y_pos;

} MAPPING_DATA;

typedef struct
{
    bool isRover;
    unsigned char centerX;
    unsigned char centerY;
    unsigned char width;
    unsigned char length;
} object;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MAPPING_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    MAPPING_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void MAPPING_Initialize ( void );


/*******************************************************************************
  Function:
    void MAPPING_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    MAPPING_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void MAPPING_Tasks( void );


#endif /* _MAPPING_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

