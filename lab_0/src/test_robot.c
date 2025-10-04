#undef __cplusplus

//-------------------------- Standard library prototypes ----------------------------------------------------
#include <stdlib.h>                 // standard functions and constant
#include <stdio.h>                  // i/o functions
#include <math.h>                   // math functions
#include <string.h>                 // string functions
#include <ctype.h>                  // character functions
#include <stdbool.h>                // bool definitions
#include "constants.h"              // program constants and structure definitions

//-------------------------------- Robot Definitions and Function Prototypes --------------------------------
int initializeRobot(void);          // creates a TCP/IP connection between this program and the robot.
int sendRobotCommand(const char *); // sends a command remotely to the SCARA robot
void closeRobot(void);              // closes the TCP/IP connection to the robot

//-----------------------------------------------------------------------------------------------------------
void main(void)
{
   if(!initializeRobot()) exit(0);

   sendRobotCommand("MOTOR_SPEED LOW\n");
   sendRobotCommand("PEN_COLOR 255 0 255\n");
   sendRobotCommand("ROTATE_JOINT ANG1 150.0 ANG2 0.0\n");

   sendRobotCommand("PEN_COLOR 0 0 0\n");
   sendRobotCommand("ROTATE_JOINT ANG1 150.0 ANG2 -170.0\n");

   sendRobotCommand("HOME\n");

   printf("Test complete...\n");
   printf("You should have seen the SCARA robot draw a series of arks.");

   exit(0);
}


