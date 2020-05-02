/**
 * @\file   ultrasonic.h
 * @\author Sorabh Gandhi, Sanju Prakash Kannioth
 * @\brief  This files contains the function declarations for the ultrasonic sensor
 * @\date   05/02/2020
 * References : http://wiringpi.com/pins/
 *              https://www.piprojects.xyz/ultrasonic-distance-sensor/
 *              
 */


#ifndef _ULTRASONIC_H
#define _ULTRASONIC_H

#include <wiringPi.h>

#define TRIG 5
#define ECHO 6


/**
--------------------------------------------------------------------------------------------
ultrasonic_setup
--------------------------------------------------------------------------------------------
*   This function is used to setup the ultrasonic sensor
*
*   @\param         void 					
*
*   @\return        void
*
*/
void ultrasonic_setup();

/**
--------------------------------------------------------------------------------------------
get_dist_in_cm
--------------------------------------------------------------------------------------------
*   This function returns the distance in cm
*
*   @\param         void 					
*
*   @\return        int   distance in cm
*
*/
int get_dist_in_cm();

/**
--------------------------------------------------------------------------------------------
Service_2
--------------------------------------------------------------------------------------------
*   Thread callback function for the ultrasonic task
*
*   @\param         threadp 					
*
*   @\return        void
*
*/
void *Service_2(void *threadp);

#endif
