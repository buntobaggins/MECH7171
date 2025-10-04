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
const unsigned char ERROR_SYMBOL = 4;     // solid black diamond symbol
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
int initializeRobot();              // creates a TCP/IP connection between this program and the robot.
int sendRobotCommand(const char *); // sends a command string to the robot
void closeRobot();                  // shuts down the connection to the robot

//----------------------------- Function Prototypes ---------------------------------------------------------
bool flushInputBuffer();         // flushes any characters left in the standard input buffer
void waitForEnterKey();          // waits for the Enter key to be pressed
void endProgram();               // ends the program from anywhere in the code
double degToRad(double);         // returns angle in radians from input angle in degrees
double radToDeg(double);         // returns angle in degrees from input angle in radians
void setPenPos(int pos);         // sets the pen position of the robot
void printRepeatedChar(unsigned char ch, int numRepeats);   // prints an ascii character repeatedly
void rotateRobotJoints(double theta1Deg, double theta2Deg); // rotates robot arms to specified joint angles
double mapAndConvertAngle(double theta); // maps radians angle into -PI <= theta <= +PI, converts to degrees
bool doAgain();  // asks the user if they want to draw another line
void printHeader(int tableWidth, const char *strTableTitle);  // prints a table header

void printTableHBorder(unsigned char chL, unsigned char chR, int tableWidth); // prints table horz. border

void printInputData(double xA, double yA, double xB, double yB, int NP);  // prints input data table

// moves the robot to a specific point (if possible) based on the reachstate value and angle values
void moveRobot(double theta1Ldeg, double theta2Ldeg, double theta1Rdeg, double theta2Rdeg, int reachState);

// prints the points data table for the current line
void printPointData(int iPoint, int NP, double x, double y,
   double theta1Ldeg, double theta2Ldeg, double theta1Rdeg, double theta2Rdeg, int reachState);

//----- TASK2 FUNCTIONS ------

// computes joint angles for given tooltip position.  Returns reachability errors packed into an int
int inverseKinematics(double x, double y, 
                      double *ptheta1LDeg, double *ptheta2Ldeg, double *ptheta1Rdeg, double *ptheta2Rdeg);

void getLineEndpoints(double *pxA, double *pyA, double *pxB, double *pyB); // gets line endpoints from user
int getNP();   // gets the number of points on a line from the user

// gets all the data necessary to draw a line with robot.  Includes endpoints and number of points on line
void getInputData(double *pxA, double *pyA, double *pxB, double *pyB, int *pNP);

void drawLine(double xA, double yA, double xB, double yB, int NP);  // draws a straight line with robot
void printLineData(double xA, double yA, double xB, double yB, int NP); // prints line data into a table

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  C program to draw straight lines with the SCARA robot
// ARGUMENTS:    none
// RETURN VALUE: an int that tells the O/S how the program ended.  0 = EXIT_SUCCESS = normal termination
int main()
{
   double xA = NAN, yA = NAN, xB = NAN, yB = NAN;  // line endpoint coordinates
   int NP = -1;                // number of points on the line, including endpoints

   if(!initializeRobot()) exit(0);  // start the robot

   sendRobotCommand("CYCLE_PEN_COLORS ON\n");  // set the pen to change colors on each move 

   // draw multiple lines using the robot
   do
   {
      setPenPos(PEN_UP); // get ready to draw the next line (move to start point with pen up)

      getInputData(&xA, &yA, &xB, &yB, &NP); // get the line input data from the user

      // clear screen for table printing
      printf("\nPress ENTER to clear the screen and print all input/output data...");
      waitForEnterKey();
      system("cls");

      printInputData(xA, yA, xB, yB, NP); // print the line input data

      drawLine(xA, yA, xB, yB, NP);       // draws a line using robot

      printLineData(xA, yA, xB, yB, NP);  // prints line point data in table
   }
   while(doAgain());  // doAgain asks the user if they want to generate another line

   sendRobotCommand("HOME\n");  // send the robot home

   closeRobot();  // close connection to robot
   endProgram();  // prompt the user to press ENTER to end the program
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Asks the user if they want to draw another line
// ARGUMENTS:    none
// RETURN VALUE: true if they want to, false if not
bool doAgain()
{
   int iret = -1;             // scanf_s return value
   bool bHasGarbage = false;  // flushInputBuffer return value
   char ch = 0;               // store single character user input

   while(true)  // ensure good clean data
   {
      // prompt the user and store function return values to check for error
      printf("\nDo you want to draw another line [y/n]? "); 
      iret = scanf_s("%c", &ch, 1);
      bHasGarbage = flushInputBuffer();

      // clean data
      if(iret == 1 && !bHasGarbage)
      {
         // convert input to true/false if possible.  If not, try again
         ch = (char)toupper(ch);  
         if(ch == 'Y')
            return true;
         else if(ch == 'N')
            return false;
         else
            printf("please enter 'y' or 'n'\n");
      }
      else  // multiple characters entered
      {
         printf("please enter a single character.\n");
      }
   }
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Move the robot using, if possible, the left arm.  If not possible, then moves the robot
//               using the right arm, if possible.  If neither arm is possible, then the robot only receives
//               a PEN_UP command.  If the robot is moved, the pen is lower _after_ the move.
//               NOTE: This can lead to redundant PEN_DOWN commands when moving the robot.
// ARGUMENTS:    theta1Ldeg, theta2Ldeg:   Left arm configuration joint angles
//               theta1Rdeg, theta2Rdeg:  Right arm configuration joint angles
//               reachState: indicates if L or theta limits are exceeded
// RETURN VALUE: none
void moveRobot(double theta1Ldeg, double theta2Ldeg, double theta1Rdeg, double theta2Rdeg, int reachState)
{
   bool bLeftOk = false, bRightOk = false, bLOk = false; // auxiliary variables to simplify code

   bLOk = !(reachState & L_EXCEEDS_MAX) && !(reachState & L_EXCEEDS_MIN); // true if L within limits

   // true if angles within limits
   bLeftOk = !(reachState & THETA1L_EXCEEDS_MAX) && !(reachState & THETA2L_EXCEEDS_MAX);
   bRightOk = !(reachState & THETA1R_EXCEEDS_MAX) && !(reachState & THETA2R_EXCEEDS_MAX);

   if(!bLOk || (!bLeftOk && !bRightOk)) // bad L or bad left/right arms
   {
      setPenPos(PEN_UP);  // left the pen because we couldn't move 
   }
   else // moveable!
   {
      if(bLeftOk) // good left so must do if can
      {
         rotateRobotJoints(theta1Ldeg, theta2Ldeg);
      }
      else // right must be ok because of above checks
      {
         rotateRobotJoints(theta1Rdeg, theta2Rdeg);
      }
      setPenPos(PEN_DOWN);  // set the pen down now that we moved
   }
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Prints the input data table
// ARGUMENTS:    xA,yA,xB,yB:  The start point (A) and end point (B)
//               NP: The number of points on the line
// RETURN VALUE: none
void printInputData(double xA, double yA, double xB, double yB, int NP)
{
   int numChars = -1;   // number of characters printed by printf

   printHeader(INPUTS_TABLE_WIDTH, strInputsTableHeader);  // print the header portion of the table

   // start point data
   numChars = printf("%c%*cStart Point: x,y = [%+*.*le, %+*.*le]",
      VL, LEFT_MARGIN, ' ', FIELD_WIDTH, PRECISION, xA, FIELD_WIDTH, PRECISION, yA);
   printf("%*c\n", INPUTS_TABLE_WIDTH - numChars, VL);

   // end point data
   numChars = printf("%c%*cEnd Point:   x,y = [%+*.*le, %+*.*le]",
      VL, LEFT_MARGIN, ' ', FIELD_WIDTH, PRECISION, xB, FIELD_WIDTH, PRECISION, yB);
   printf("%*c\n", INPUTS_TABLE_WIDTH - numChars, VL);

   // NP data
   numChars = printf("%c%*cNP = %d", VL, LEFT_MARGIN, ' ', NP);
   printf("%*c\n", INPUTS_TABLE_WIDTH - numChars, VL);

   // bottom border
   printTableHBorder(BL, BR, INPUTS_TABLE_WIDTH);
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Prints a table header of a table.  Includes the top/bottom borders of the title.  
//               Title text is centered horizontally.
// ARGUMENTS:    tableWidth:  The width of the table in characters, including the left/right borders
//               strTableTitle: pointer to a string that holds the title text
// RETURN VALUE: none
void printHeader(int tableWidth, const char *strTableTitle)
{
   int numChars = -1;   // number of characters printed by printf
   int titleWidth = -1; // number of characters in the table title

   titleWidth = (int)strlen(strTableTitle);  // get number of characters in the title

   // top border
   printTableHBorder(TL, TR, tableWidth);

   // Header title.  Title string is centered.
   numChars = printf("%c%*c%s", VL, (tableWidth - 2 - titleWidth) / 2, ' ', strTableTitle);
   printf("%*c\n", tableWidth - numChars, VL);

   // mid border
   printTableHBorder(CL, CR, tableWidth);
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Prints the points data table
// ARGUMENTS:    iPoint: index of point on the line
//               NP: number of points on the line
//               x,y: line point cartesian coordinates
//               theta1Ldeg through theta2Rdeg:  Left and right joint angles in degrees
//               reachState:  value to represent all error conditions
// RETURN VALUE: ????
void printPointData(int iPoint, int NP, double x, double y, double theta1Ldeg, double theta2Ldeg,
   double theta1Rdeg, double theta2Rdeg, int reachState)
{
   int numChars = -1;      // number of characters returned by printf     
   char chL = 0, chR = 0;  // left and right edge characters for table horizontal borders

   // print x,y, and L
   numChars = printf("%c%*cPOINT %02d:   x = %+*.*le,   y = %+*.*le.   L = %+*.*le", VL, LEFT_MARGIN, ' ', iPoint + 1,
      FIELD_WIDTH, PRECISION, x, FIELD_WIDTH, PRECISION, y, FIELD_WIDTH, PRECISION, sqrt(x * x + y * y));
   printf("%*c\n", OUTPUTS_TABLE_WIDTH - numChars, VL);

   if(reachState == L_EXCEEDS_MIN || reachState == L_EXCEEDS_MAX)  // L error messages
   {
      numChars = printf("%c%*c", VL, LEFT_MARGIN, ' ');

      printRepeatedChar(ERROR_SYMBOL, NUM_ERROR_SYMBOLS);
      numChars += NUM_ERROR_SYMBOLS;

      if(reachState == L_EXCEEDS_MAX)  // L > LMAX
         numChars += printf(" POINT IS OUTSIDE MAXIMUM REACH OF ROBOT (L_MAX = %lg mm) ", LMAX);
      else // L < LMIN
         numChars += printf(" POINT IS INSIDE MINIMUM REACH OF ROBOT (L_MIN = %lg mm) ", LMIN);

      printRepeatedChar(ERROR_SYMBOL, NUM_ERROR_SYMBOLS);
      numChars += NUM_ERROR_SYMBOLS;

      printf("%*c\n", OUTPUTS_TABLE_WIDTH - numChars, VL);
   }
   else  // print angles and any associated error messages
   {
      //----- left arm -----
      numChars = printf("%c%*cLEFT ARM:  %c1 = %+*.*le%c, %c2 = %+*.*le%c.  ", VL, LEFT_MARGIN, ' ',
         THETA_SYMBOL, FIELD_WIDTH, PRECISION, theta1Ldeg, DEGREE_SYMBOL,
         THETA_SYMBOL, FIELD_WIDTH, PRECISION, theta2Ldeg, DEGREE_SYMBOL);

      // erros
      if(reachState & THETA1L_EXCEEDS_MAX || reachState & THETA2L_EXCEEDS_MAX)
      {
         if(reachState & THETA1L_EXCEEDS_MAX && reachState & THETA2L_EXCEEDS_MAX) // both angles bad
            numChars += printf("%c1 and %c2 exceed max angle!", THETA_SYMBOL, THETA_SYMBOL);
         else if(reachState & THETA1L_EXCEEDS_MAX) // shoulder angle bad
            numChars += printf("%c1 exceeds max angle!", THETA_SYMBOL);
         else // elbow angle bad
            numChars += printf("%c2 exceeds max angle!", THETA_SYMBOL);
      }
      printf("%*c\n", OUTPUTS_TABLE_WIDTH - numChars, VL);

      //----- right arm -----
      numChars = printf("%c%*cRIGHT ARM: %c1 = %+*.*le%c, %c2 = %+*.*le%c.  ", VL, LEFT_MARGIN, ' ',
         THETA_SYMBOL, FIELD_WIDTH, PRECISION, theta1Rdeg, DEGREE_SYMBOL,
         THETA_SYMBOL, FIELD_WIDTH, PRECISION, theta2Rdeg, DEGREE_SYMBOL);

      // errors
      if(reachState & THETA1R_EXCEEDS_MAX || reachState & THETA2R_EXCEEDS_MAX) // both angles bad
      {
         if(reachState & THETA1R_EXCEEDS_MAX && reachState & THETA2R_EXCEEDS_MAX)
            numChars += printf("%c1 and %c2 exceed max angle!", THETA_SYMBOL, THETA_SYMBOL);
         else if(reachState & THETA1R_EXCEEDS_MAX) // shoulder angle bad
            numChars += printf("%c1 exceeds max angle!", THETA_SYMBOL);
         else // elbow angle bad
            numChars += printf("%c2 exceeds max angle!", THETA_SYMBOL);
      }
      printf("%*c\n", OUTPUTS_TABLE_WIDTH - numChars, VL);
   }

   // print border beneath angle data.  Note that last point is different than the others
   chL = iPoint == NP - 1 ? BL : CL;
   chR = iPoint == NP - 1 ? BR : CR;
   printTableHBorder(chL, chR, OUTPUTS_TABLE_WIDTH);
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Prints a horiztonal border for a table
// ARGUMENTS:    chL/chR: left/right edge characters
//               tableWidth: with of table including borders
// RETURN VALUE: none
void printTableHBorder(unsigned char chL, unsigned char chR, int tableWidth)
{
   printf("%c", chL);  // left edge
   printRepeatedChar(HL, tableWidth - 2); // long horizontal line
   printf("%c\n", chR); // right edge
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Computes the SCARA joint angles for a given tooltip x,y coordinate point.  Packs an int with
//               errors associated with physical and joint angle limits being exceeded.
// ARGUMENTS:    x,y:  tooltip coordinate point
//               ptheta1Ldeg, ptheta2Ldeg:  pointers to left arm joint angles
//               ptheta1Rdeg, ptheta2Rdeg:  pointers to right arm joint angles
// RETURN VALUE: an int containing bits that are set based and L reachability and joint angle limits exceeded
int inverseKinematics(double x, double y, 
                      double *ptheta1Ldeg, double *ptheta2Ldeg, double *ptheta1Rdeg, double *ptheta2Rdeg)
{
   int reachState =  -1;                      // int to be packed with bitwize error flags
   double L = NAN, beta = NAN, alpha = NAN;  // L = tooltip distance.  beta/alpha = auxiliary angles
   double theta1 = NAN, theta2 = NAN;        // auxiliary variables to compute joint angles in radians

   // initialize joint angles
   *ptheta1Ldeg = *ptheta2Ldeg = *ptheta1Rdeg = *ptheta2Rdeg = NAN;

   //------ compute L and check for reachability
   L = sqrt(x * x + y * y);  // tooltip distance

   if(L < LMIN) return L_EXCEEDS_MIN;  // L too small
   
   if(L > LMAX) return L_EXCEEDS_MIN;  // L too large

   reachState = 0; // set return value starting value (0 means both arms can reach)

   // compute auxiliary angles alpha and beta
   beta = atan2(y, x);
   alpha = acos((L2 * L2 - L * L - L1 * L1) / (-2.0 * L * L1));

   //--- left arm angles
   theta1 = beta + alpha;
   theta2 = atan2(y - L1 * sin(theta1), x - L1 * cos(theta1)) - theta1;
   // map the angles into range -180 deg <= theta <= +180 deg
   *ptheta1Ldeg = mapAndConvertAngle(theta1);
   *ptheta2Ldeg = mapAndConvertAngle(theta2);
   // set the reach state for each joint
   if(fabs(*ptheta1Ldeg) > ABS_THETA1_DEG_MAX) reachState |= THETA1L_EXCEEDS_MAX;
   if(fabs(*ptheta2Ldeg) > ABS_THETA2_DEG_MAX) reachState |= THETA2L_EXCEEDS_MAX;

   //--- right arm angles
   theta1 = beta - alpha;
   theta2 = atan2(y - L1 * sin(theta1), x - L1 * cos(theta1)) - theta1;
   // map the angles into range -180 <= theta <= +180
   *ptheta1Rdeg = mapAndConvertAngle(theta1);
   *ptheta2Rdeg = mapAndConvertAngle(theta2);
   // set the reach state for each joint
   if(fabs(*ptheta1Rdeg) > ABS_THETA1_DEG_MAX) reachState |= THETA1R_EXCEEDS_MAX;
   if(fabs(*ptheta2Rdeg) > ABS_THETA2_DEG_MAX) reachState |= THETA2R_EXCEEDS_MAX;

   return reachState;
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Gets the cartesian coordinates of the each endpoint on a line
// ARGUMENTS:    pxA, pyA: pointers to line start point cartesian coordinates
//               pxB, pyB: pointers to line end point cartesian coordinates
// RETURN VALUE: 
void getLineEndpoints(double *pxA, double *pyA, double *pxB, double *pyB)
{
   int iret = -1; // scanf_s return value
   bool bHasGarbage = false;  // flushInputBuffer return value

   while(true)  // re-ask if input data has any errors
   {
      // prompt user and get function feedback values to trap errors
      printf("Enter the start and point coordinates [x0,y0,x1,y1] (comma separated): ");
      iret = scanf_s("%lf , %lf , %lf , %lf", pxA, pyA, pxB, pyB);
      bHasGarbage = flushInputBuffer();

      // good data, adios loop!
      if(iret == 4 && !bHasGarbage)
      {
         printf("Got 4 good values (%lg, %lg, %lg, %lg)!\n", *pxA, *pyA, *pxB, *pyB);
         return;
      }

      //--- now check for user input errors
      if(iret == 4) // must have trailing garbage on yB
      {
         printf("Got all values (%lg, %lg, %lg, %lg) but yB contains trailing garbage!\n\n", *pxA, *pyA, *pxB, *pyB);
      }
      else if(iret == 3)  // Could be trailing garbage on xB or leading garbage on yB or no comma
      {
         printf("Got first 3 values (%lg, %lg, %lg)\n", *pxA, *pyA, *pxB);
         printf("xB may have trailing garbage or yB may have leading garbage or you forgot the comma.\n\n");
      }
      else if(iret == 2)  // Could be trailing garbage on yA or leading garbage on xB or no comma
      {
         printf("Got first 2 values (%lg, %lg)\n", *pxA, *pyA);
         printf("yA may have trailing garbage or xB may have leading garbage or you forgot the comma.\n\n");
      }
      else if(iret == 1)  // Could be trailing garbage on xA or leading garbage on yA or no comma
      {
         printf("Only got xA value (%lg)!\n", *pxA);
         printf("xA may have trailing garbage or yA may have leading garbage or you forgot the comma.\n\n");
      }
      else // iret == 0.  Got nothing.  Must be leading garbage on xA value
      {
         printf("Didn't get any usable data. xA value contains leading non-numerical characters.\n\n");
      }
   }
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Gets, from the user, the number of points used to generate a straight line of equally 
//               spaced points.  Data is ensured to be error free.
// ARGUMENTS:    none
// RETURN VALUE: the number of points
int getNP()
{
   int NP = -1;   // number of points of the line
   int iret = -1; // scanf_s return value
   bool bHasGarbage = false;  // flushInputBuffer return value

   while(true)  // re-ask if input data has any errors
   {
      // prompt user and get function feedback values to trap errors
      printf("Enter the number of points NP on the line (>1, includes endpoints): ");
      iret = scanf_s("%d", &NP);
      bHasGarbage = flushInputBuffer();

      // got clean data, check if in range
      if(iret == 1 && !bHasGarbage)
      {
         if(NP > 1)  // all good, adios loop!
         {
            printf("Got a good NP value (%d).  Thanks!!\n", NP);
            return NP;
         }
         else // data out of range!
         {
            printf("NP value (%d) must be >1!\n", NP);
         }
      }
      else if(iret == 0)  // got nothing
      {
         printf("Didn't get any usable data. Input contains leading non-numerical characters.\n\n");
      }
      else // must have trailing garbage on NP
      {
         printf("Got NP value (%d) but it contains trailing non-numerical characters!\n\n", NP);
      }
   }
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Gets all the data necessary to draw an approximation of a straight line using the robot.
//               To approximate a straight line, the line is subdivided into a number of equally spaced
//               points, NP.  xA,yB is the line start point, xB,yB is the line end point
// ARGUMENTS:    pxA, pyA: pointers to the line start point coordinates
//               pxB, pyB: pointers to the line end point coordinates
//               pNP: pointer to the number of point on the line
// RETURN VALUE: none
void getInputData(double *pxA, double *pyA, double *pxB, double *pyB, int *pNP)
{
   getLineEndpoints(pxA, pyA, pxB, pyB);  // get the line endpoint coordinates from the user
   *pNP = getNP();                        // get the number of points on the line from the user
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Draws an approximation of a straight line using the robot.  To approximate a straight line, 
//               the line is subdivided into a number of equally spaced points and the robot is
//               moved from point to point. 
// ARGUMENTS:    xA, yA: line start point coordinates
//               xB, yB: line end point coordinates
//               NP: number of points on the line
// RETURN VALUE: none
void drawLine(double xA, double yA, double xB, double yB, int NP)
{
   double theta1Ldeg = NAN, theta2Ldeg = NAN, theta1Rdeg = NAN, theta2Rdeg = NAN; // joint angles in degrees
   double t = NAN;            // parametric line parameter
   double x = NAN, y = NAN;   // point coordinates
   int iPoint = 0;            // index of point on line
   int reachState = -1;       // reachability state for both arms for given tooltip point x,y

   // loop to generate points and move the robot
   for(iPoint = 0; iPoint < NP; iPoint++)
   {
      // generate the current point x,y data using parametric equation
      t = (double)iPoint / (double)(NP - 1);
      x = xA + (xB - xA) * t;
      y = yA + (yB - yA) * t;

      //----------------------- inverseKinematics calculations -----------------------
      reachState = inverseKinematics(x, y, &theta1Ldeg, &theta2Ldeg, &theta1Rdeg, &theta2Rdeg);

      //-------- print the point data to the table
      moveRobot(theta1Ldeg, theta2Ldeg, theta1Rdeg, theta2Rdeg, reachState);
   }
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Prints data for each point on the line drawn by the robot.  Note that the robot moves from
//               one point to another in curved arcs, so to approximate a straight line, the line is 
//               subdivided into a number of equally spaced points and the robot is moved from point to point.
// ARGUMENTS:    xA, yA: line start point coordinates
//               xB, yB: line end point coordinates
//               NP: number of points on the line
// RETURN VALUE: none
void printLineData(double xA, double yA, double xB, double yB, int NP)
{
   double theta1Ldeg = NAN, theta2Ldeg = NAN, theta1Rdeg = NAN, theta2Rdeg = NAN; // joint angles in degrees
   double t = NAN;            // parametric line parameter
   double x = NAN, y = NAN;   // point coordinates
   int iPoint = 0;            // index of point on line
   int reachState = -1;       // reachability state for both arms for given tooltip point x,y

   printHeader(OUTPUTS_TABLE_WIDTH, strLinePointsTableHeader); // print table header with borders

   // loop to generate points print each point's data to a table
   for(iPoint = 0; iPoint < NP; iPoint++)
   {
      // generate the current point x,y data using parametric equation
      t = (double)iPoint / (double)(NP - 1);
      x = xA + (xB - xA) * t;
      y = yA + (yB - yA) * t;

      //----------------------- inverseKinematics calculations -----------------------
      reachState = inverseKinematics(x, y, &theta1Ldeg, &theta2Ldeg, &theta1Rdeg, &theta2Rdeg);

      //-------- print the point data to the table
      printPointData(iPoint, NP, x, y, theta1Ldeg, theta2Ldeg, theta1Rdeg, theta2Rdeg, reachState);
   }
}


//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Maps an angle in radians into a an equivalent angle in degrees that follows the angle ranges 
//               defined in the robot (-180 <= theta <= +180)
// ARGUMENTS:    theta: the angle in radians 
// RETURN VALUE: the mapped angle in radians
double mapAndConvertAngle(double theta)
{
   theta = fmod(theta, 2.0 * PI);  // put in range -2*PI <= ang <= +2*PI

   // map into range -PI <= ang <= +PI
   if(theta > PI)
      theta -= 2.0 * PI;
   else if(theta < -PI)
      theta += 2.0 * PI;

   return radToDeg(theta);  // convert to degrees before returning
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
// DESCRIPTION:  Ends program from anywhere in code.
// ARGUMENTS:    none
// RETURN VALUE: none
void endProgram()
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