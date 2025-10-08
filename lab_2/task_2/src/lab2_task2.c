/************************************************************************************************************
Course: MECH 7171 - Engineering Programming - Fall 2024.

Program: Lab1 - Task2: Console I/O

Purpose: Modify the Lab1 Task1 program to compute the coordinates and cooresponding SCARA robot joint angles
         of NP equally spaced points on a straight line with endpoints xA,yA and xB,yB.

         C programming techniques will include variables, keyboard input (scanf_s),
         formatted output (printf), branches, and loops.

Author(s): ???? (include each of your sets)

Declaration: I/We, ????, declare that the following program was written by me/us.

Date Created: September 28 2024

************************************************************************************************************/

#define _USE_MATH_DEFINES

//-------------------------- Standard library prototypes ----------------------------------------------------
#include <stdlib.h>  // standard functions and constant
#include <stdio.h>   // i/o functions
#include <math.h>    // math functions
#include <string.h>  // string functions
#include <ctype.h>   // character functions
#include <stdbool.h> // bool definitions
#include <windows.h>

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

const int TOOLTIP_COORDINATES_COLUMN_WIDTH = 31;  // value includes left/right borders!!!!
#define TOOLTIP_COORDINATES_COLUMN_WIDTH 31
const int JOINT_ANGLES_COLUMN_WIDTH = 88;         // value includes left/right borders!!!!
#define JOINT_ANGLES_COLUMN_WIDTH 88
const int LEFT_MARGIN = 2;    // all tables have left margin which is number of spaces between
                              // left border and the first character printed in a table row

// Table header strings
const char *strTooltipCoordinateHeader = "Tool Tip Coordinates [mm]";  // tooltip coordinate header
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
   int numChars = -1;                              // stores the number of characters printed by printf
   int iret = -1;                                  // stores integer return value from scanf_s
   bool bHasGarbage = false;                       // stores the integer return value from flushInputBuffer
   bool bWithinLLimits = false;                    // set to true if L is in the limits LMIN <= L <= LMAX
   int i = -1, n = -1;                             // loop counters
   double xA = NAN, yA = NAN, xB = NAN, yB = NAN;  // straight line endpoint coordinates
   int NP = -1;                                    // number of points on the straight line
   double t = NAN;                                 // parametric variable for producing points on the line
   double x = NAN, y = NAN;                        // point coordinates
   double L = NAN;                                 // distance to tooltip
   double alpha = NAN, beta = NAN, theta2a = NAN;  // auxiliary angle in joint angles calculations
   double theta1L = NAN, theta2L = NAN;            //  left arm shoulder, elbow angles (radians)
   double theta1R = NAN, theta2R = NAN;            // right arm shoulder, elbow angles (radians)
   double theta1Ldeg = NAN, theta2Ldeg = NAN;      //  left arm shoulder, elbow angles (radians)
   double theta1Rdeg = NAN, theta2Rdeg = NAN;      // right arm shoulder, elbow angles (radians)

   // length of table header strings
   int tooltipCoordinateHeaderLength = (int)strlen(strTooltipCoordinateHeader);
   int jointAnglesHeaderLength = (int)strlen(strJointAnglesHeader);

   //---------- get the line endpoint coordinates.  Don't stop asking until have good, clean data
   while(true)
   {
      printf("Please enter your two sets of cordinates and the number of point (xA yA xB yB) \n");
      iret = scanf_s("%lf %lf %lf %lf",&xA,&yA,&xB,&yB);
      bHasGarbage = flushInputBuffer();
      if (iret == 4)
      {
         printf("%lf %lf %lf %lf \n",xA,yA,xB,yB);
         break;
      } else {
         printf("Invalid input. Please try again. \n");
      }
   }

   //---------- get NP
   while(true)
   {
      printf("Please enter the number of points (NP) \n");
      iret = scanf_s("%d",&NP);
      bHasGarbage = flushInputBuffer();
      if (bHasGarbage == 0 && NP >= 2)
      {
         printf("%d \n",NP);
         break;
      } else {
         printf("Invalid input. Please try again. \n");
      }
   }

   //---------
   printf("Generating table of line points data...\n\n");

   // print the table guide

   //======================== Compute joint angles data for each point on the line  =========================
   //======================== Place values in a two column table                    =========================
   printGuide(TOOLTIP_COORDINATES_COLUMN_WIDTH+JOINT_ANGLES_COLUMN_WIDTH);

   for(i=1;i<=TOOLTIP_COORDINATES_COLUMN_WIDTH+JOINT_ANGLES_COLUMN_WIDTH;i++){
   switch (i)
      {
      case 1:
         printf("%c",TL);
         break;
      case TOOLTIP_COORDINATES_COLUMN_WIDTH:
         printf("%c",TC);
         break;
      case TOOLTIP_COORDINATES_COLUMN_WIDTH+JOINT_ANGLES_COLUMN_WIDTH:
         printf("%c\n",TR);
         break;
      default:
         printf("%c",HL);
         break;
      }
   }
   printf("%-*c",LEFT_MARGIN+1,VL);
   printf("%-*s",TOOLTIP_COORDINATES_COLUMN_WIDTH-4, strTooltipCoordinateHeader);
   printf("%c",VL);
   printf("%*s",((JOINT_ANGLES_COLUMN_WIDTH-1-jointAnglesHeaderLength)/2)+jointAnglesHeaderLength, strJointAnglesHeader);
   printf("%*c\n",((JOINT_ANGLES_COLUMN_WIDTH+1-jointAnglesHeaderLength)/2),VL);

   for(i=1;i<=TOOLTIP_COORDINATES_COLUMN_WIDTH+JOINT_ANGLES_COLUMN_WIDTH;i++){
      switch (i)
      {
      case 1:
         printf("%c",CL);
         break;
      case TOOLTIP_COORDINATES_COLUMN_WIDTH:
         printf("%c",CC);
         break;
      case TOOLTIP_COORDINATES_COLUMN_WIDTH+JOINT_ANGLES_COLUMN_WIDTH:
         printf("%c\n",CR);
         break;
      default:
         printf("%c",HL);
         break;
      }
   }

   for(i=1;i<=NP;i++){
      t = (i-1.0)/(NP-1);
      x = ((1 - t)*xA) + (xB*t);
      y = ((1 - t)*yA) + (yB*t);
      L = sqrt((x*x) + (y*y));

      beta = atan2(y,x);
      alpha = acos((L2*L2)-(L*L)-(L1*L1)/(-2*L*L1));

      theta1L = beta + alpha;
      theta1R = beta - alpha;

      theta2L = atan2(y-L1*sin(theta1L),x-L1*cos(theta1L))-theta1L;
      theta2R = atan2(y-L1*sin(theta1R),x-L1*cos(theta1R))-theta1R;

      theta1Ldeg = radToDeg(mapAngle(theta1L));
      theta1Rdeg = radToDeg(mapAngle(theta1R));
      theta2Ldeg = radToDeg(mapAngle(theta2L));
      theta2Rdeg = radToDeg(mapAngle(theta2R));

      numChars = printf("%-*c",LEFT_MARGIN+1,VL);
      numChars += printf("x,y = %+*.*lf, %+*.*lf",FIELD_WIDTH,PRECISION,x,FIELD_WIDTH,PRECISION,y);
      printf("%*c",TOOLTIP_COORDINATES_COLUMN_WIDTH-numChars,VL);
      if (L<LMIN){
         bWithinLLimits == false;
         numChars=printf("%*c!!!!! POINT IS INSIDE MINIMUM REACH OF THE ROBOT (L = %.*lf [mm]) !!!!!",LEFT_MARGIN,32,PRECISION,LMIN);
      } else if (L>LMAX){
         bWithinLLimits == false;
         numChars=printf("%*c!!!!! POINT IS OUTSIDE MAXIMUM REACH OF THE ROBOT (L = %.*lf [mm]) !!!!!",LEFT_MARGIN,32,PRECISION,LMAX);
      } else {
         bWithinLLimits == true;
         numChars=printf("%*cLeft Arm Shoulder Angle  ",LEFT_MARGIN,32);
         numChars+=printf("= %+*.*lf",FIELD_WIDTH,PRECISION,theta1Ldeg);
         if (theta1L > ABS_THETA1_DEG_MAX){
            numChars+=printf("    *** MAXIMUM ANGLE (%c%lf%c) EXCEEDED! ***    ",PLUSMINUS_SYMBOL,ABS_THETA1_DEG_MAX,DEGREE_SYMBOL);
         }
      }

      printf("%*c\n",JOINT_ANGLES_COLUMN_WIDTH-numChars,VL);

      numChars = printf("%-*c",LEFT_MARGIN+1,VL);
      numChars += printf("  L = %+*.*lf",FIELD_WIDTH,PRECISION,L);
      printf("%*c",TOOLTIP_COORDINATES_COLUMN_WIDTH-numChars,VL);

      if(bWithinLLimits==true){
         numChars=printf("%*cLeft Arm Elbow Angle  ",LEFT_MARGIN,32);
         numChars+=printf("= %+*.*lf",FIELD_WIDTH,PRECISION,theta2Ldeg);
         if (theta2L > ABS_THETA2_DEG_MAX){
            numChars+=printf("    *** MAXIMUM ANGLE (%c%lf%c) EXCEEDED! ***    ",PLUSMINUS_SYMBOL,ABS_THETA2_DEG_MAX,DEGREE_SYMBOL);
         }
      }

      printf("%*c\n",JOINT_ANGLES_COLUMN_WIDTH,VL);

      if (i == NP) {
         for(n=1;n<=TOOLTIP_COORDINATES_COLUMN_WIDTH+JOINT_ANGLES_COLUMN_WIDTH;n++){
            switch (n)
            {
            case 1:
               printf("%c",BL);
               break;
            case TOOLTIP_COORDINATES_COLUMN_WIDTH:
               printf("%c",BC);
               break;
            case TOOLTIP_COORDINATES_COLUMN_WIDTH+JOINT_ANGLES_COLUMN_WIDTH:
               printf("%c\n",BR);
               break;
            default:
               printf("%c",HL);
               break;
            }
         }
      } else {
         for(n=1;n<=TOOLTIP_COORDINATES_COLUMN_WIDTH+JOINT_ANGLES_COLUMN_WIDTH;n++){
            switch (n)
            {
            case 1:
               printf("%c",CL);
               break;
            case TOOLTIP_COORDINATES_COLUMN_WIDTH:
               printf("%c",CC);
               break;
            case TOOLTIP_COORDINATES_COLUMN_WIDTH+JOINT_ANGLES_COLUMN_WIDTH:
               printf("%c\n",CR);
               break;
            default:
               printf("%c",HL);
               break;
            }
         }
      }
   }


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
   printf("This program computes NP equally spaced tooltip coordinate points along a straight line.  These\n");
   printf("points can be used to draw an approximation of a straight line using the robot.  The higher the\n");
   printf("value of NP, the less wavey the line will appear. The user enters the line start point (xA, yA),\n");
   printf("end point (xB, yB), and the number of points NP.  The program calculates the coordinates of the\n");
   printf("points and displays them in a table.\n\n");
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
   int i;

   for(i=1;i<length;i++){
      if(i>=100 && i%10 == 0){
         printf("%d",i/100);
      } else{printf(" ");}
   }
   printf("\n");
   for(i=1;i<length;i++){
      if(i>=10 && i%10 == 0){
         printf("%d",(i/10)%10);
      } else{printf(" ");}
   }
   printf("\n");
   for(i=1;i<=length;i++){
      printf("%d",i%10);
   }
   printf("\n");
}