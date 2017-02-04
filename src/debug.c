/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */
#include "debug.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

/* ************************************************************************** */
/** Descriptive Data Item Name

  @Summary
    Brief one-line summary of the data item.
    
  @Description
    Full description, explaining the purpose and usage of data item.
    <p>
    Additional description in consecutive paragraphs separated by HTML 
    paragraph breaks, as necessary.
    <p>
    Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.
    
  @Remarks
    Any additional remarks
 */
int global_data;


/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

    void dbgOutputVal(unsigned char outVal){
        //TRISCCLR = 0x0008;
        //ODCCCLR = 0x0008;
        TRISFCLR = 0x0003;
        ODCFCLR = 0x0003;
        TRISGCLR = 0x0300;
        ODCGCLR = 0x0300;
        TRISDCLR = 0x0940;
        ODCDCLR = 0x0940;
        TRISACLR = 0x0400;
        ODCACLR = 0x0400;
        
        LATDCLR = 0x0940;
        unsigned short D = 0x0000;
        D |= (outVal & 0x01) << 6;
        D |= (outVal & 0x08) << 8;
        D |= (outVal & 0x10) << 4;
        LATDSET = D;
        
        LATACLR = 0x0400;
        LATASET = (outVal & 0x02) << 9;
        
        //LATCCLR = 0x0008;
        //LATCSET = (outVal & 0x02) << 2;
        
        LATFCLR = 0x0003;
        //unsigned short B = 0x0000;
        //B |= (outVal & 0x04) << 1;
        //B |= (outVal & 0xC0) >> 6;
        LATFSET = (outVal & 0xC0) >> 6;
        
        LATGCLR = 0x0300;
        unsigned short G = 0x0000;
        G |= (outVal & 0x04) << 6;
        //G |= (outVal & 0x08);
        //G |= (outVal & 0x10) >> 2;
        G |= (outVal & 0x20) << 4;
        LATGSET = G;
        
        /*TRISFCLR = 0x0002;
        ODCFCLR = 0x0002;
        TRISDCLR = 0x0940;
        ODCDCLR = 0x0940;
        TRISGCLR = 0x03C0;
        ODCGCLR = 0x03C0;
        
        LATFCLR = 0x0002;
        LATFSET = (outVal & 0x01) << 1;
        
        LATDCLR = 0x0940;
        char D = 0x00;
        D |= (outVal & 0x02) << 5;
        D |= (outVal & 0x04) << 6;
        D |= (outVal & 0x08) << 8;
        LATDSET = D;
        
        LATGCLR = 0x03C0;
        char G = 0x00;
        G |= (outVal & 0x30) << 3;
        G |= (outVal & 0x40);
        G |= (outVal & 0x80) << 2;
        LATGSET = G;*/
        
        /*TRISDCLR = 0x00FF;
        ODCDCLR  = 0x00FF;
        
        LATDCLR = 0x00FF;
        LATDSET = outVal;*/
    }
    
    void dbgUARTVal(unsigned char outVal){
        //if(!(DRV_USART_TRANSFER_STATUS_TRANSMIT_FULL & DRV_USART0_TransferStatus()) ){
            DRV_USART0_WriteByte (outVal);
        //}else{
        //    printf("hi");
        //}
    }
    
    void dbgOutputLoc(unsigned char outVal){
        TRISECLR = 0x00FF;
        ODCECLR  = 0x00FF;
        
        LATECLR = 0x00FF;
        LATESET = outVal;
    }
    
    void dbgPauseAll(){
        dbgOutputLoc(DBG_LOC_PAUSE_ALL);
        //Turn off all interrupts
        //Turn on a LED
        while(1){;}
    }

/*  A brief description of a section can be given directly below the section
    banner.
 */

/* ************************************************************************** */

/** 
  @Function
    int ExampleLocalFunctionName ( int param1, int param2 ) 

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
static int ExampleLocalFunction(int param1, int param2) {
    return 0;
}


/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

// *****************************************************************************

/** 
  @Function
    int ExampleInterfaceFunctionName ( int param1, int param2 ) 

  @Summary
    Brief one-line description of the function.

  @Remarks
    Refer to the example_file.h interface header for function usage details.
 */
int ExampleInterfaceFunction(int param1, int param2) {
    return 0;
}


/* *****************************************************************************
 End of File
 */
