/************************************************************************************************************
Course: MECH 7171 - Engineering Programming - Fall 2024.

Program: Lab1 - Task1: Console I/O, decisions, and loops

Purpose: To develop your first C program.  The program will ask the user for a SCARA Robot tooltip x,y
         coordinate.  The program will check the coordinate for L reachability based on inner/outer arm
         lengths L1/L2.  If reachable, it will calculate the left/right arm joint angles necessary to
         position the tooltip.  The data will be printed into two tables:

         1) The tooltip x,y coordinates.

         2) L and the  calculated joint angles.  If L is outside the range LMIN <= L <= LMAX, an error
            message will be printed indicating which limit was exceeded.  In this case, no angle data
            is printed (because the angles are not possible to calculate).  If L is within limits, _ALL_
            angles will be printed.  If any angle is outside the associated limits, it must be appended
            with an error message.

         C programming techniques will include variables, keyboard input (scanf_s),
         formatted output (printf), branches, and loops.

Author(s): ???? (include each of your sets)

Declaration: I/We, ????, declare that the following program was written by me/us.

Date Created: September 27 2024

************************************************************************************************************/

#define _USE_MATH_DEFINES


//-------------------------- Standard library prototypes ----------------------------------------------------
#include <stdlib.h>  // standard functions and constant
#include <stdio.h>   // i/o functions
#include <math.h>    // math functions
#include <string.h>  // string functions
#include <ctype.h>   // character functions
#include <stdbool.h> // bool definitions

//---------------------------- Program Constants ------------------------------------------------------------
const double PI = M_PI;                   // the one and only
const double L1 = 350.0;                  // length of the inner arm
const double L2 = 250.0;                  // length of the outer arm
const double ABS_THETA1_DEG_MAX = 150.0;  // maximum magnitude of shoulder angle in degrees
const double ABS_THETA2_DEG_MAX = 170.0;  // maximum magnitude of elbow angle in degrees

// can't use const doubles for L_MAX and L_MIN because global const variables require 
// constant expressions, i.e., something without a memory address
#define LMAX (L1 + L2) // max L -> maximum reach of robot
#define LMIN (L1 - L2) // min L -> minimum reach of robot (no angle restrictions)

const int PRECISION = 2;      // for printing x,y,L values to console
const int FIELD_WIDTH = 7;    // for printing values to console

const int TOOLTIP_COORDINATES_COLUMN_WIDTH = 51;  // value includes left/right borders!!!!
const int JOINT_ANGLES_COLUMN_WIDTH = 88;         // value includes left/right borders!!!!
const int LEFT_MARGIN = 2;  // all tables have left margin which is number of spaces between 
                            // left border and the first character printed in a table row

// Table header strings
const char *strTooltipCoordinateHeader = "Tool Tip Coordinates";  // tooltip coordinate header
const char *strJointAnglesHeader = "Joint Angles Analysis";       // joint angle analysis header

// TABLE BORDER SYMBOLS
const unsigned char HL = 196;  // horizontal border line
const unsigned char VL = 179;  // vertical border line
const unsigned char TL = 218;  // top left border symbol
const unsigned char TC = 194;  // top center border symbol
const unsigned char TR = 191;  // top right border symbol
const unsigned char CL = 195;  // left center border symbol
const unsigned char CC = 197;  // center center border symbol (cross)
const unsigned char CR = 180;  // right center border symbol
const unsigned char BL = 192;  // bottom left border symbol
const unsigned char BC = 193;  // bottom center border symbol
const unsigned char BR = 217;  // bottom right border symbol

const unsigned char DEGREE_SYMBOL = 248;  // degree symbol
const unsigned char PLUSMINUS_SYMBOL = 241;  // degree symbol

//----------------------------- Function Prototypes ---------------------------------------------------------
bool flushInputBuffer();         // flushes any characters left in the standard input buffer
void waitForEnterKey();          // waits for the Enter key to be pressed
void introduction();             // prints the program introduction
double degToRad(double angDeg);  // input an angle in degrees and output an angle in radians
double radToDeg(double ang);     // input an angle in radians and output an angle in degrees
double mapAngle(double ang);     // input an angle (radians) and map into range -PI <= ang <= +PI
void printGuide(int length);     // prints column guide

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  C program to ask the user for a tooltip coordinate (x,y) and analyze the associated 
//               joint angles. The tooltip coordinates and the joint angle analysis data will be printed in 
//               separate formatted tables.
// ARGUMENTS:    none
// RETURN VALUE: an int that tells the O/S how the program ended.  0 = EXIT_SUCCESS = normal termination
int main()
{
   int numChars = -1;                     // stores the number of characters printed by printf
   int iret = -1;                         // stores integer return value from scanf_s
   bool bHasGarbage = false;              // stores the integer return value from flushInputBuffer
   bool bWithinLLimits = false;           // // set to true if L is in the limits LMIN <= L <= LMAX
   int i = -1;                            // loop counter
   double x = NAN, y = NAN;               // point coordinates
   double L = NAN;                        // distance to tooltip
   double alpha = NAN, beta = NAN, theta2a = NAN;  // auxiliary angle in joint angles calculations
   double theta1L = NAN, theta2L = NAN;            //  left arm shoulder, elbow angles (radians)
   double theta1R = NAN, theta2R = NAN;            // right arm shoulder, elbow angles (radians)
   double theta1Ldeg = NAN, theta2Ldeg = NAN;      //  left arm shoulder, elbow angles (radians)
   double theta1Rdeg = NAN, theta2Rdeg = NAN;      // right arm shoulder, elbow angles (radians)

   // length of table header strings
   int tooltipCoordinateHeaderLength = (int)strlen(strTooltipCoordinateHeader);
   int jointAnglesHeaderLength = (int)strlen(strJointAnglesHeader);

   introduction();  // print the program introduction

   //============== get the tooltip x,y values.  Don't stop asking until have good, clean data ==============
   while(true)
   {
      break;
   }


   //========================= Compute Joint Angles Data =========================



   printf("\nPress ENTER to clear the screen and print all input/output data..."); // ready to print tables
   waitForEnterKey();
   system("cls");

   //========================= Print Input Data =========================
   printGuide(TOOLTIP_COORDINATES_COLUMN_WIDTH);





   //========================= Print Joint Angles Analysis Data =========================
   printGuide(JOINT_ANGLES_COLUMN_WIDTH);




   printf("\nPress ENTER to end the program...\n");
   waitForEnterKey();
   return (EXIT_SUCCESS);
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  This function flushes the input buffer to avoid scanf issues
// ARGUMENTS:    none
// RETURN VALUE: false if nothing or only '\n' in stdin. true if extra keystrokes precede the '\n'.
//               Good for detecting left over garbage from scanf_s in the input buffer
bool flushInputBuffer()
{
   int ch; // temp character variable
   bool bHasGarbage = false;

   // exit loop when all characters are flushed
   while((ch = getchar()) != '\n' && ch != EOF)
   {
      if(!bHasGarbage) bHasGarbage = true;
   }
   return bHasGarbage;
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Waits for user to press enter.  flushes stdin if keystrokes precede enter
// ARGUMENTS:    none
// RETURN VALUE: none
void waitForEnterKey()
{
   unsigned char ch;
   if((ch = (unsigned char)getchar()) != EOF && ch != '\n') flushInputBuffer();
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  converts an angle from degrees to radians
// ARGUMENTS:    angDeg:  the input angle in degrees
// RETURN VALUE: the angle converted to radians
double degToRad(double angDeg)
{
   return (PI / 180.0) * angDeg;
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  converts an angle from radians to degrees
// ARGUMENTS:    ang:  the input angle in radians
// RETURN VALUE: the angle converted to degrees
double radToDeg(double ang)
{
   return (180.0 / PI) * ang;
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  maps an input angle in radians to the range -PI <= ang <= +PI
// ARGUMENTS:    ang:  the input angle in radians
// RETURN VALUE: the mapped angle in radians
double mapAngle(double ang)
{
   // if ang > PI, keep subtracting 2*PI until the angle is in range
   while(ang > PI)
   {
      ang -= 2.0 * PI;
   }

   // if ang < -PI, keep adding 2*PI until the angle is in range
   while(ang < -PI)
   {
      ang += 2.0 * PI;
   }

   return ang;
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  prints an explanation of the program
// ARGUMENTS:    none
// RETURN VALUE: none
void introduction()
{
   double LMIN_ACTUAL = sqrt(L1 * L1 + L2 * L2 - 2.0 * L1 * L2 * cos(PI - degToRad(ABS_THETA2_DEG_MAX)));
   printf("This program asks the user for SCARA robot tooltip coordinates (x,y) and does an analysis\n");
   printf("of the associated joint angles.  The robot has an inner arm L1 = %.1lf [mm] and an outer\n", L1);
   printf("arm L2 = %.1lf[mm].  Accordingly, the minimum possible reach of the robot, L1-L2, is %.1lf[mm]\n",
      L2, LMIN);
   printf("and the maximum reach, L1+L2, is L = %.1lf[mm].  The robot is further constrained by having\n",
      LMAX);
   printf("a maximum shoulder angle magnitude of %.1lf%c and a maximum elbow angle magnitude of %.1lf%c.\n",
      ABS_THETA1_DEG_MAX, DEGREE_SYMBOL, ABS_THETA2_DEG_MAX, DEGREE_SYMBOL);
   printf("This elbow angle constraint will give an actual minimum reach of %.*lf[mm].\n",
      PRECISION, LMIN_ACTUAL);
   printf("\nPress ENTER to begin...\n");
   waitForEnterKey();
}


//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  prints column guide in the form:
// 
//          1         2         3         4         5         6         7
// 1234567890123456789012345678901234567890123456789012345678901234567890
// 
// ARGUMENTS:    the length of the column guide in characters
// RETURN VALUE: none
void printGuide(int length)
{
}

