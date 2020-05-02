/**
 * @\file   motor_control.h
 * @\author Steve Kennedy, Sanju Prakash Kannioth
 * @\brief  This files contains the function declarations for motor control
 * @\date   05/02/2020
 * References : http://wiringpi.com/pins/
 *              https://www.instructables.com/id/DC-Motor-Control-With-Raspberry-Pi-and-L293D/
 *
 */

#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H

#include <wiringPi.h>

#define MOTOR_A_IN1 2
#define MOTOR_A_IN2 0

#define MOTOR_B_IN1 12
#define MOTOR_B_IN2 13

/**
--------------------------------------------------------------------------------------------
l293d_setup
--------------------------------------------------------------------------------------------
*   This function is used to setup the l293d motor driver
*
*   @\param         void 					
*
*   @\return        void
*
*/
void l293d_setup();


/**
--------------------------------------------------------------------------------------------
motor_forward
--------------------------------------------------------------------------------------------
*   Function to move the robot forward
*
*   @\param         void 					
*
*   @\return        void
*
*/
void motor_forward();


/**
--------------------------------------------------------------------------------------------
motor_reverse
--------------------------------------------------------------------------------------------
*   Function to move the robot backward
*
*   @\param         void 					
*
*   @\return        void
*
*/
void motor_reverse();



/**
--------------------------------------------------------------------------------------------
motor_right
--------------------------------------------------------------------------------------------
*   Function to turn the robot right
*
*   @\param         void 					
*
*   @\return        void
*
*/
void motor_right();


/**
--------------------------------------------------------------------------------------------
motor_left
--------------------------------------------------------------------------------------------
*   Function to turn the robot left
*
*   @\param         void 					
*
*   @\return        void
*
*/
void motor_left();


/**
--------------------------------------------------------------------------------------------
motor_stop
--------------------------------------------------------------------------------------------
*   Function to stop the robot
*
*   @\param         void 					
*
*   @\return        void
*
*/
void motor_stop();


/**
--------------------------------------------------------------------------------------------
Service_1
--------------------------------------------------------------------------------------------
*   Thread callback function for the motor task
*
*   @\param         threadp 					
*
*   @\return        void
*
*/
void *Service_1(void *threadp);
#endif
