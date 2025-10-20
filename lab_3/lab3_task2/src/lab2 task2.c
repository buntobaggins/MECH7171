/************************************************************************************************************
Course: MECH 7171 - Engineering Programs

Program: Lab2 Task1: C program to draw straight lines with the SCARA robot using functions Part 1

Details: Compute the [x,y] coordinates of points distributed on a straight line.  The coordinates of each
         point are based on a parametric equation.  Check each point for L and theta reachability according
         to the constraints of the SCARA robot.  Use the SCARA robot simulator to draw an approximation of a
         straight line composed of multiple points.  Components of the C program include variables, formatted
         console output, user input with error checking, branches, loops, functions, and bitwise operations.

Author(s): Walker Golembioski (Set A)

Declaration: I, Walker Golembioski, declare that the following program was written by me.

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
const unsigned char ERROR_SYMBOL = 219;     // left  pointing black arrowhead symbol
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
void printInputData(double xA,double yA,double xB,double yB,int NP);  // prints some stuff I guess
void printRowBorder(int length, int style); // Print the border between rows for a given length, style 1 top of table, style 2 middle, style 3 bottom
void printPointData(double x,double y, double theta1Ldeg, double theta2Ldeg, double theta1Rdeg, double theta2Rdeg, int i, double L, int reachState);
void moveRobot(int reachState,double theta1Ldeg, double theta2Ldeg, double theta1Rdeg, double theta2Rdeg, int i);
void getInputData(double *xA,double *xB,double *yA,double *yB,int *NP);
void printLineData(double xA,double yA,double xB,double yB,int NP);
void getLineEndPoints(double *xA,double *xB,double *yA,double *yB);

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  C program to draw straight lines with the SCARA robot
// ARGUMENTS:    none
// RETURN VALUE: an int that tells the O/S how the program ended.  0 = EXIT_SUCCESS = normal termination
int main(void)
{
   double xA = NAN, yA = NAN, xB = NAN, yB = NAN;  // line endpoints
   int NP = -1;  // number of points on the line, including endpoints


   if(!initializeRobot()) exit(0);  // start the robot

   sendRobotCommand("CYCLE_PEN_COLORS ON\n");  // set the pen to change colors on each move



   do
   {
      setPenPos(PEN_UP); // get ready to draw the next line (move to start point with pen up)
      //===================================  Get the input data (inline) ===================================
      getInputData(&xA,&xB,&yA,&yB,&NP);

      printf("\nPress ENTER to clear the screen and print all input/output data...");
      waitForEnterKey();
      system("cls");
      //==============================  Print the input data table (function) ==============================
      printInputData(xA,yA,xB,yB,NP);
      //============= Generate the line points and move the robot to each reachable point.     =============
      //============= If the point is reachable, move to it with the pen down.  If it is not,  =============
      //============= move to the next reachable pont with the pen up and then put it down.    =============
      drawLine(xA,yA,xB,yB,NP);
      //============= Print the results in a table as shown in the specification               =============
      printLineData(xA,yA,xB,yB,NP);
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
   char ch;
   while (true)
   {
      printf("Do you want to draw another line [y/n]? ");
      scanf_s("%c", &ch, 1);
      flushInputBuffer();
      if (tolower(ch) == 'y')
      {
         doAgain = true;
         break;
      } else if(tolower(ch) == 'n'){
         doAgain = false;
         break;
      } else {
         printf("Invalid input please try again\n");
      }
   }


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
   int numChars;
   int length = strlen(strHeaderTitle);
   numChars=printf("%c",VL);
   numChars+=printf("%*s",((tableWidth-2-length)/2)+length,strHeaderTitle);
   printf("%*c\n",tableWidth-numChars,VL);
}

void printInputData(double xA,double yA,double xB,double yB,int NP)
{
   int numChars;
   printRowBorder(INPUTS_TABLE_WIDTH,1);
   printHeader(INPUTS_TABLE_WIDTH,strInputsTableHeader);
   printRowBorder(INPUTS_TABLE_WIDTH,2);
   numChars=printf("%-*c",1+LEFT_MARGIN, VL);
   numChars+=printf("Start Point: x,y = [%+*.*lf, %+*.*lf]",FIELD_WIDTH,PRECISION,xA,FIELD_WIDTH,PRECISION,yA);
   printf("%*c\n",INPUTS_TABLE_WIDTH-numChars,VL);
   numChars=printf("%-*c",1+LEFT_MARGIN, VL);
   numChars+=printf("End Point:   x,y = [%+*.*lf, %+*.*lf]",FIELD_WIDTH,PRECISION,xB,FIELD_WIDTH,PRECISION,yB);
   printf("%*c\n",INPUTS_TABLE_WIDTH-numChars,VL);
   numChars=printf("%-*c",1+LEFT_MARGIN, VL);
   numChars+=printf("NP = %d",NP);
   printf("%*c\n",INPUTS_TABLE_WIDTH-numChars,VL);
   printRowBorder(INPUTS_TABLE_WIDTH,3);
}

void printRowBorder(int length, int style)
{
   //I know this is bad but it works and I'm to lazy to change it on both files
   int i;
   switch (style)
   {
   case 1:
      for(i=1;i<=length;i++){
         if (i == 1){
            printf("%c",TL);
         } else if (i == length){
            printf("%c\n",TR);
         } else{
            printf("%c",HL);
         }
      }
      break;
   case 2:
      for(i=1;i<=length;i++){
         if (i == 1){
            printf("%c",CL);
         } else if (i == length){
            printf("%c\n",CR);
         } else{
            printf("%c",HL);
         }
      }
      break;
   case 3:
      for(i=1;i<=length;i++){
         if (i == 1){
            printf("%c",BL);
         } else if (i == length){
            printf("%c\n",BR);
         } else{
            printf("%c",HL);
         }
      }
      break;

   default:
      break;
   }
}
void printPointData(double x,double y, double theta1Ldeg, double theta2Ldeg, double theta1Rdeg, double theta2Rdeg, int i, double L, int reachState){
   int numChars;
   numChars=printf("%-*c",1+LEFT_MARGIN, VL);
   numChars+=printf("POINT %02d:",i);
   numChars+=printf("   x = %+*.*lf,",FIELD_WIDTH,PRECISION,x);
   numChars+=printf("   y = %+*.*lf.",FIELD_WIDTH,PRECISION,y);
   numChars+=printf("   L = %+*.*lf",FIELD_WIDTH,PRECISION,L);
   printf("%*c\n",OUTPUTS_TABLE_WIDTH-numChars,VL);

   if((reachState & L_EXCEEDS_MAX) == L_EXCEEDS_MAX){
      numChars=printf("%-*c",1+LEFT_MARGIN, VL);
      printRepeatedChar(ERROR_SYMBOL,NUM_ERROR_SYMBOLS);
      numChars+=printf(" POINT IS OUTSIDE MAXIMUM REACH OF ROBOT (L_MAX = %.0lf) ",LMAX);
      printRepeatedChar(ERROR_SYMBOL,NUM_ERROR_SYMBOLS);
      printf("%*c\n",OUTPUTS_TABLE_WIDTH-numChars-(2*NUM_ERROR_SYMBOLS),VL);
   } else if((reachState & L_EXCEEDS_MIN) == L_EXCEEDS_MIN){
      numChars=printf("%-*c",1+LEFT_MARGIN, VL);
      printRepeatedChar(ERROR_SYMBOL,NUM_ERROR_SYMBOLS);
      numChars+=printf(" POINT IS INSIDE MINIMUM REACH OF ROBOT (L_MIN = %.0lf) ",LMIN);
      printRepeatedChar(ERROR_SYMBOL,NUM_ERROR_SYMBOLS);
      printf("%*c\n",OUTPUTS_TABLE_WIDTH-numChars-(2*NUM_ERROR_SYMBOLS),VL);
   } else{
      numChars=printf("%-*c",1+LEFT_MARGIN, VL);
      numChars+=printf("LEFT ARM:");
      numChars+=printf("  %c1 = %+*.*lf%c,",THETA_SYMBOL,FIELD_WIDTH,PRECISION,theta1Ldeg,DEGREE_SYMBOL);
      numChars+=printf(" %c2 = %+*.*lf%c.  ",THETA_SYMBOL,FIELD_WIDTH,PRECISION,theta2Ldeg,DEGREE_SYMBOL);
      if(((reachState & THETA1L_EXCEEDS_MAX) == THETA1L_EXCEEDS_MAX)&&((reachState & THETA2L_EXCEEDS_MAX) == THETA2L_EXCEEDS_MAX)){
         numChars+=printf("%c1 and %c2 exceeds max angle!",THETA_SYMBOL,THETA_SYMBOL);
      } else if(((reachState & THETA1L_EXCEEDS_MAX) == THETA1L_EXCEEDS_MAX)){
         numChars+=printf("%c1 exceeds max angle!",THETA_SYMBOL);
      } else if(((reachState & THETA2L_EXCEEDS_MAX) == THETA2L_EXCEEDS_MAX)){
         numChars+=printf("%c2 exceeds max angle!",THETA_SYMBOL);
      }
      printf("%*c\n",OUTPUTS_TABLE_WIDTH-numChars,VL);

      numChars=printf("%-*c",1+LEFT_MARGIN, VL);
      numChars+=printf("RIGHT ARM:");
      numChars+=printf(" %c1 = %+*.*lf%c,",THETA_SYMBOL,FIELD_WIDTH,PRECISION,theta1Rdeg,DEGREE_SYMBOL);
      numChars+=printf(" %c2 = %+*.*lf%c.  ",THETA_SYMBOL,FIELD_WIDTH,PRECISION,theta2Rdeg,DEGREE_SYMBOL);
      if(((reachState & THETA1R_EXCEEDS_MAX) == THETA1R_EXCEEDS_MAX)&&((reachState & THETA2R_EXCEEDS_MAX) == THETA2R_EXCEEDS_MAX)){
         numChars+=printf("%c1 and %c2 exceeds max angle!",THETA_SYMBOL,THETA_SYMBOL);
      } else if((reachState & THETA1R_EXCEEDS_MAX) == THETA1R_EXCEEDS_MAX){
         numChars+=printf("%c1 exceeds max angle!",THETA_SYMBOL);
      } else if((reachState & THETA2R_EXCEEDS_MAX) == THETA2R_EXCEEDS_MAX){
         numChars+=printf("%c2 exceeds max angle!",THETA_SYMBOL);
      }
      printf("%*c\n",OUTPUTS_TABLE_WIDTH-numChars,VL);
   }
}

void moveRobot(int reachState,double theta1Ldeg, double theta2Ldeg, double theta1Rdeg, double theta2Rdeg, int i){
   if((reachState & (L_EXCEEDS_MAX | L_EXCEEDS_MIN)) == 0){
      if((reachState & (THETA1L_EXCEEDS_MAX | THETA2L_EXCEEDS_MAX)) == 0){
         rotateRobotJoints(theta1Ldeg,theta2Ldeg);
         setPenPos(PEN_DOWN);
      } else if((reachState & (THETA1R_EXCEEDS_MAX | THETA2R_EXCEEDS_MAX)) == 0){
         rotateRobotJoints(theta1Rdeg,theta2Rdeg);
         setPenPos(PEN_DOWN);
      } else {
         setPenPos(PEN_UP);
      }
   } else {
      setPenPos(PEN_UP);
   }
}
void getInputData(double *xA,double *xB,double *yA,double *yB,int *NP){
   getLineEndPoints(xA,xB,yA,yB);
   *NP=getNP();
}
void getLineEndPoints(double *xA,double *xB,double *yA,double *yB){
      // line endpoint data (xA,yA,xB,yB)
      int iret,bHasGarbage;
      while(true)
         {
            printf("Enter the line end point coordinates xA,yA,xB,yB (comma seperated): ");
            iret = scanf_s("%lf,%lf,%lf,%lf",xA,yA,xB,yB);
            bHasGarbage = flushInputBuffer();
            if (iret == 4)
            {
               printf("Got good end point values: xA, yA = [%.0lf,%.0lf] and xB, yB = [%.0lf,%.0lf].\n",*xA,*yA,*xB,*yB);
               break;
            } else {
               printf("Invalid input. Please try again. \n");
            }
         }
}
int getNP(){
      // number of points on the line (NP)
      int iret,bHasGarbage,NP;
      while(true)
         {
            printf("Enter the number of points on the line, NP: ");
            iret = scanf_s("%d",&NP);
            bHasGarbage = flushInputBuffer();
            if (bHasGarbage == 0 && NP >= 2)
            {
               printf("Got good value: NP = %d.\n",NP);
               break;
            } else {
               printf("Invalid input. Please try again. \n");
            }
         }
         return NP;
}
int inverseKinematics(double x,double y,double *theta1Ldeg,double *theta2Ldeg,double *theta1Rdeg,double *theta2Rdeg,double L){
   int reachState =0;

   if(L>LMAX){
      reachState=(reachState | L_EXCEEDS_MAX);
      *theta1Ldeg = NAN;
      *theta2Ldeg = NAN;
      *theta1Rdeg = NAN;
      *theta2Rdeg = NAN;
   } else if(L<LMIN){
      reachState=(reachState | L_EXCEEDS_MIN);
      *theta1Ldeg = NAN;
      *theta2Ldeg = NAN;
      *theta1Rdeg = NAN;
      *theta2Rdeg = NAN;
   }else{

      double beta = atan2(y,x);
      double alpha = acos(((L2*L2)-(L*L)-(L1*L1))/(-2*L*L1));

      double theta1 = beta + alpha;
      *theta1Ldeg = mapAndConvertAngle(theta1);
      *theta2Ldeg = mapAndConvertAngle(atan2(y-L1*sin(theta1),x-L1*cos(theta1))-theta1);
      if(fabs(*theta1Ldeg)>ABS_THETA1_DEG_MAX){
         reachState=(reachState | THETA1L_EXCEEDS_MAX);
      }
      if(fabs(*theta2Ldeg)>ABS_THETA2_DEG_MAX){
         reachState=(reachState | THETA2L_EXCEEDS_MAX);
      }

      theta1 = beta - alpha;
      *theta1Rdeg = mapAndConvertAngle(theta1);
      *theta2Rdeg = mapAndConvertAngle(atan2(y-L1*sin(theta1),x-L1*cos(theta1))-theta1);
      if(fabs(*theta1Rdeg)>ABS_THETA1_DEG_MAX){
         reachState=(reachState | THETA1R_EXCEEDS_MAX);
      }
      if(fabs(*theta2Rdeg)>ABS_THETA2_DEG_MAX){
         reachState=(reachState | THETA2R_EXCEEDS_MAX);
      }
   }

   return reachState;
}
void printLineData(double xA,double yA,double xB,double yB,int NP){
   int iPoint,reachState;
   double theta1Ldeg,theta2Ldeg,theta1Rdeg,theta2Rdeg;
   printRowBorder(OUTPUTS_TABLE_WIDTH,1);
   printHeader(OUTPUTS_TABLE_WIDTH,strLinePointsTableHeader);
   printRowBorder(OUTPUTS_TABLE_WIDTH,2);
   for(iPoint=1;iPoint<=NP;iPoint++){
      double t = (iPoint-1.0)/(NP-1);
      double x = ((1 - t)*xA) + (xB*t);
      double y = ((1 - t)*yA) + (yB*t);
      double L = sqrt((x*x) + (y*y));
      reachState = inverseKinematics(x,y,&theta1Ldeg,&theta2Ldeg,&theta1Rdeg,&theta2Rdeg,L);
      printPointData(x,y,theta1Ldeg,theta2Ldeg,theta1Rdeg,theta2Rdeg,iPoint,L,reachState);
      if(iPoint==NP){
            printRowBorder(OUTPUTS_TABLE_WIDTH,3);
         } else {
            printRowBorder(OUTPUTS_TABLE_WIDTH,2);
         }
   }

}
drawLine(double xA,double yA,double xB,double yB,int NP){
   int iPoint,reachState;
   double theta1Ldeg,theta2Ldeg,theta1Rdeg,theta2Rdeg;
   for(iPoint=1;iPoint<=NP;iPoint++){
      double t = (iPoint-1.0)/(NP-1);
      double x = ((1 - t)*xA) + (xB*t);
      double y = ((1 - t)*yA) + (yB*t);
      double L = sqrt((x*x) + (y*y));
      reachState = inverseKinematics(x,y,&theta1Ldeg,&theta2Ldeg,&theta1Rdeg,&theta2Rdeg,L);
      //printf("\n %lf %lf %lf %lf %d\n",theta1Ldeg,theta2Ldeg,theta1Rdeg,theta2Rdeg,reachState); <- used for testing
      moveRobot(reachState,theta1Ldeg,theta2Ldeg,theta1Rdeg,theta2Rdeg,iPoint);
   }
}
