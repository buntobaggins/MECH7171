/**********************************************************************************************************************
Course: MECH 7171 - Engineering Programming

Program: Lab 3: Structures

Details: Control the SCARA robot using various commands.  Draw lines and curves with various colors and line
         thicknesses. The line or curve will only be drawn if it can be drawn in its entirety using
         the left arm or right arm without switching arms at any point.  Components of the C program include
         variables (including pointers), formatted console output, error checked user input, branches, loops,
         functions, bitwise operations, structures.

Author(s): <names/ids>

Declaration: I/We, <names>, declare that the following program was written by me/us.

Date Created: Oct 25 2024

**********************************************************************************************************************/

#undef __cplusplus

//-------------------------- Standard library prototypes --------------------------------------------------------------
#include <stdlib.h>                             // standard functions and constant
#include <stdio.h>                              // i/o functions
#include <math.h>                               // math functions
#include <string.h>                             // string functions
#include <ctype.h>                              // character functions
#include <stdbool.h>                            // bool definitions
#include <stdarg.h>                             // for variable argument functions
#include <share.h>                              // for file sharing
#include "constants.h"                          // program constants and structure definitions
#include "lab5_task1.h"                         // your lab4 function prototypes
#include "helper_functions.h"                   // function prototypes for helper functions given by Dave for lab 5

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  C program to draw various shapes with the SCARA robot
// ARGUMENTS:    none
// RETURN VALUE: an int that tells the O/S how the program ended.  0 = EXIT_SUCCESS = normal termination
int main(void) {
   int iShape = -1;  // shape index (LINE/ARC/QUADRATIC_BEZIER)

   if(!initializeRobot()) exit(0);  // connect to the robot

   if(!openLogFile()) endProgram(NULL);

   do  // draw shapes until user terminates
   {
      iShape = getShapeChoice(); // get users choice of shape to draw
      drawShape(iShape);         // draw requested shape
   }
   while(doAgain());

   // set final position
   setPenPos(PEN_UP);
   setMotorSpeed(MOTOR_SPEED_HIGH);
   sendRobotCommand("HOME\n");

   fclose(m_flog);
   endProgram(NULL);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  draws a shape with a robot according to user specifications
// ARGUMENTS:    shape: index of the shape to draw (LINE/ARC/BEZIER)
// RETURN VALUE: none
void drawShape(int shape) {

}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Asks the user what shape they want to draw
// ARGUMENTS:    none
// RETURN VALUE: shape choice index (see enum SHAPES for values)
int getShapeChoice(void) {
   return LINE; // placeholder return value
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  sets the pen position of the robot
// ARGUMENTS:    penPos:  the pen position index (PEN_UP or PEN_DOWN)
// RETURN VALUE: none
void setPenPos(int penPos) {

}


//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  sets the robot motor speed
// ARGUMENTS:    motorSpeed:  the motor speed index (HIGH\MEDIUM\LOW)
// RETURN VALUE: none
void setMotorSpeed(int motorSpeed) {
}

