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
#include "linesensing.h"

//This function returns whether or not a color is blue given 16-bit RGBC values
bool ColorIsBlue(unsigned short red, unsigned short green, unsigned short blue, unsigned short clear){
    int threshold = (unsigned short) (clear / 16);
    int gDiff = blue - green;
    int rDiff = blue - red;
    if (clear < 50){
        //Return 0 if we are looking at black/nothing
        return 0;
    }
    return ((gDiff >= threshold) && (rDiff >= threshold));
}

/* *****************************************************************************
 End of File
 */