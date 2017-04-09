/* ************************************************************************** */
/** Descriptive File Name
  @Company
    Company Name
  @File Name
    filename.h
  @Summary
    Brief description of the file.
  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef LINESENSING_H    /* Guard against multiple inclusion */
#define LINESENSING_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "debug.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    //This function returns whether or not a color is blue given 16-bit RGBC values
    bool ColorIsBlue(unsigned short red, unsigned short green, unsigned short blue, unsigned short clear);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* LINESENSING_H */

/* *****************************************************************************
 End of File
 */