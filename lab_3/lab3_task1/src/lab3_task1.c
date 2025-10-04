/************************************************************************************************************
Course: MECH 7171 - Engineering Programs

Program: Lab2 Task1: C program to draw straight lines with the SCARA robot using functions Part 1

Details: Compute the [x,y] coordinates of points distributed on a straight line.  The coordinates of each
         point are based on a parametric equation.  Check each point for L and theta reachability according
         to the constraints of the SCARA robot.  Use the SCARA robot simulator to draw an approximation of a
         straight line composed of multiple points.  Components of the C program include variables, formatted
         console output, user input with error checking, branches, loops, functions, and bitwise operations.

Author(s): ????

Declaration: I/We, ??????, declare that the following program was written by me/us.

Date Created: October 10 2024

************************************************************************************************************/
#undef __cplusplus

#define _USE_MATH_DEFINES

//-------------------------- Standard library prototypes ----------------------------------------------------
#include <stdlib.h>  // standard functions and constant
#include <stdio.h>   // i/o functions
#include <math.h>    // math functions
#include <string.h>  // string functions
#include <ctype.h>   // character functions
#include <stdbool.h> // bool definitions

//---------------------------- Program Constants ------------------------------------------------------------
const double PI = M_PI;             // the one and only

const double L1 = 350.0;                     // length of the inner arm
const double L2 = 250.0;                     // length of the outer arm
const double ABS_THETA1_DEG_MAX = 150.0;     // maximum magnitude of shoulder angle in degrees
const double ABS_THETA2_DEG_MAX = 170.0;     // maximum magnitude of elbow angle in degrees
#define LMAX (L1 + L2)                       // max L -> maximum reach of robot
#define LMIN (L1 - L2)                       // min L -> minimum reach of robot

const int PRECISION = 4;         // for printing values to console
const int FIELD_WIDTH = 9;       // for printing values to console

const int OUTPUTS_TABLE_WIDTH = 85; // Table width for output values (value includes left/right borders)
const int INPUTS_TABLE_WIDTH = 50;  // Table width for input values (value includes left/right borders)
const int LEFT_MARGIN = 2;          // number of spaces between left border and the first character printed

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

const unsigned char DEGREE_SYMBOL = 248;  // the degree symbol
const unsigned char THETA_SYMBOL = 233;   // theta symbol
const unsigned char ERROR_SYMBOL = 4;     // left  pointing black arrowhead symbol
const int NUM_ERROR_SYMBOLS = 5;          // number of leading/trailing error symbols to print

// constants to indicate reach errors
const int L_EXCEEDS_MIN = 1 << 1;         // L < LMIN
const int L_EXCEEDS_MAX = 1 << 2;         // L > LMAX
const int THETA1L_EXCEEDS_MAX = 1 << 3;   // |theta1LDeg| > ABS_THETA1_DEG_MAX
const int THETA2L_EXCEEDS_MAX = 1 << 4;   // |theta2LDeg| > ABS_THETA2_DEG_MAX
const int THETA1R_EXCEEDS_MAX = 1 << 5;   // |theta1RDeg| > ABS_THETA1_DEG_MAX
const int THETA2R_EXCEEDS_MAX = 1 << 6;   // |theta2RDeg| > ABS_THETA2_DEG_MAX

// Inputs table header string
const char *strInputsTableHeader = "Program Input Values";
// Line point data table header string
const char *strLinePointsTableHeader = "Line Point Data";

enum ARRAY_SIZES { MAX_COMMAND_SIZE = 256 };  // size of char array used to store SCARA command strings
enum PEN_POS { PEN_UP, PEN_DOWN };            // indexes of the SCARA pen position

//-------------------------------- Robot Definitions and Function Prototypes --------------------------------
int initializeRobot(void);              // creates a TCP/IP connection between this program and the robot.
int sendRobotCommand(const char *); // sends a command string to the robot
void closeRobot(void);                  // shuts down the connection to the robot

//----------------------------- Function Prototypes ---------------------------------------------------------
bool flushInputBuffer(void);         // flushes any characters left in the standard input buffer
void waitForEnterKey(void);          // waits for the Enter key to be pressed
void endProgram(void);               // ends the program from anywhere in the code
double degToRad(double);         // returns angle in radians from input angle in degrees
double radToDeg(double);         // returns angle in degrees from input angle in radians
void setPenPos(int pos);         // sets the pen position of the robot
void printRepeatedChar(unsigned char ch, int numRepeats);   // prints an ascii character repeatedly
void rotateRobotJoints(double theta1Deg, double theta2Deg); // rotates robot arms to specified joint angles
double mapAndConvertAngle(double theta); // maps angle in radians into degrees in the range -180 to +180
bool doAgain(void); // asks the user if they want do draw another line
void printHeader(int tableWidth, const char *strHeaderTitle);  // prints a table header

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  C program to draw straight lines with the SCARA robot
// ARGUMENTS:    none
// RETURN VALUE: an int that tells the O/S how the program ended.  0 = EXIT_SUCCESS = normal termination
int main(void)
{
   double theta1Ldeg = NAN, theta2Ldeg = NAN, theta1Rdeg = NAN, theta2Rdeg = NAN; // joint angles in degrees
   double theta1 = NAN, theta2 = NAN;              // auxiliary variables for inverseKinematic calcs
   double xA = NAN, yA = NAN, xB = NAN, yB = NAN;  // line endpoints
   double t = NAN;                                 // parametric line parameter
   double x = NAN, y = NAN;                        // point coordinates
   double alpha = NAN, beta = NAN;                 // inverseKinematics auxiliary angles
   double L;                                       // distance to tooltip
   int NP = 0;                // number of points on the line, including endpoints
   int iPoint = 0;            // index of point on line
   int reachState = -1;       // reachability state for both arms for given tooltip point x,y
   int iret = -1;             // scanf_s return value
   bool bHasGarbage = false;  // flushInputBuffer return value

   if(!initializeRobot()) exit(0);  // start the robot

   sendRobotCommand("CYCLE_PEN_COLORS ON\n");  // set the pen to change colors on each move

   do
   {
      setPenPos(PEN_UP); // get ready to draw the next line (move to start point with pen up)

      //===================================  Get the input data (inline) ===================================

      // line endpoint data (xA,yA,xB,yB)



      // number of points on the line (NP)



      printf("\nPress ENTER to clear the screen and print all input/output data...");
      waitForEnterKey();
      system("cls");


      //==============================  Print the input data table (function) ==============================




      //============= Generate the line points and move the robot to each reachable point.     =============
      //============= If the point is reachable, move to it with the pen down.  If it is not,  =============
      //============= move to the next reachable pont with the pen up and then put it down.    =============
      //============= Print the results in a table as shown in the specification               =============


      // print table header with borders

      // loop to generate points and draw the line with the robot


   }
   while(doAgain());  // doAgain asks the user if they want to generate another line

   sendRobotCommand("HOME\n");  // send the robot home

   closeRobot();  // sever connection with robot
   endProgram();  // ask user to press ENTER to end the program
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Asks the user if they want to draw another line
// ARGUMENTS:    none
// RETURN VALUE: true if they want to, false if not
bool doAgain(void)
{
   bool doAgain = false;  // set to true if user wants to draw another line




   return doAgain;
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Maps an angle in radians into a an equivalent angle in degrees that follows the angle ranges
//               defined in the robot (-180 deg <= thetaDeg <= +180 deg)
// ARGUMENTS:    theta: the angle in radians
// RETURN VALUE: the mapped angle in degrees
double mapAndConvertAngle(double theta)
{
   theta = fmod(theta, 2.0 * PI);  // put in range -2*PI <= ang <= +2*PI

   // map into range -PI <= ang <= +PI
   if(theta > PI)
      theta -= 2.0 * PI;
   else if(theta < -PI)
      theta += 2.0 * PI;

   return radToDeg(theta);
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Returns angle in degrees from input angle in radian
// ARGUMENTS:    angDeg:  angle in degrees
// RETURN VALUE: angle in radians
double degToRad(double angDeg)
{
   return (PI / 180.0) * angDeg;
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Returns angle in radians from input angle in degrees
// ARGUMENTS:    angRad:  angle in radians
// RETURN VALUE: angle in degrees
double radToDeg(double angRad)
{
   return (180.0 / PI) * angRad;
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  This function flushes the input buffer to avoid scanf issues
// ARGUMENTS:    none
// RETURN VALUE: false if nothing or only '\n' in stdin. true if extra keystrokes precede the '\n'.
//               Good for detecting left over garbage from scanf_s in the input buffer
bool flushInputBuffer(void)
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
void waitForEnterKey(void)
{
   unsigned char ch;
   if((ch = (unsigned char)getchar()) != EOF && ch != '\n') flushInputBuffer();
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Ends program from anywhere in code.
// ARGUMENTS:    none
// RETURN VALUE: none
void endProgram(void)
{
   printf("\nPress ENTER to end the program...\n");
   waitForEnterKey();
   exit(0);
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Prints a character to the console repeatedly
// ARGUMENTS:    ch: character to repeat
//               numRepeats: number of repeats
// RETURN VALUE: none
void printRepeatedChar(unsigned char ch, int numRepeats)
{
   int i = -1; // loop counter

   for(i = 0; i < numRepeats; i++)
   {
      printf("%c", ch);
   }
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Move the SCARA Robot to specified joint angles
// ARGUMENTS:    theta1Deg, theta2Deg:  shoulder and elbow angles in degrees
// RETURN VALUE: none
void rotateRobotJoints(double theta1Deg, double theta2Deg)
{
   char strCommand[MAX_COMMAND_SIZE];  // for storing robot command strings

   // compose the command string using sprintf_s.  Use default (6 decimals) precision
   sprintf_s(strCommand, MAX_COMMAND_SIZE, "ROTATE_JOINT ANG1 %lf ANG2 %lf\n", theta1Deg, theta2Deg);

   // send the command to the robot
   sendRobotCommand(strCommand);
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  sets the pen position of the robot
// ARGUMENTS:    penPos:  the pen position index (PEN_UP or PEN_DOWN)
// RETURN VALUE: none
void setPenPos(int penPos)
{
   if(penPos == PEN_UP)  // set the pen to the up position
      sendRobotCommand("PEN_UP\n");
   else if(penPos == PEN_DOWN)  // set the pen to the down position
      sendRobotCommand("PEN_DOWN\n");
   else // unkown position
      printf("Bad value (%d) for pen positiion\n", penPos);
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  prints a centered header title string for a table of a specified width
// ARGUMENTS:    tableWidth:  The width of the table in characters, including the left/right borders
//               strHeaderTitle: The header title string
// RETURN VALUE: none
void printHeader(int tableWidth, const char *strHeaderTitle)
{
}