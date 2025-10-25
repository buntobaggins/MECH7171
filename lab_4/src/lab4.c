/**********************************************************************************************************************
Course: MECH 7171 - Engineering Programming

Program: Lab 3: Structures

Details: Control the SCARA robot using various commands.  Draw lines and curves with various colors and line
         thicknesses. The line or curve will only be drawn if it can be drawn in its entirety using
         the left arm or right arm without switching arms at any point.  Components of the C program include
         variables (including pointers), formatted console output, error checked user input, branches, loops,
         functions, bitwise operations, structures.

Author(s): Walker Golembioski, A01374098

Declaration: I, Walker Golembioski, declare that the following program was written by me.

Date Created: Oct 25 2024

**********************************************************************************************************************/
#undef __cplusplus

#define USE_VOID_POINTER

//-------------------------- Standard library prototypes --------------------------------------------------------------
#include <stdlib.h>                                                  // standard functions and constant
#include <stdio.h>                                                   // i/o functions
#include <math.h>                                                    // math functions
#include <string.h>                                                  // string functions
#include <ctype.h>                                                   // character functions
#include <stdbool.h>                                                 // bool definitions
#include <stdarg.h>                                                  // for variable argument functions
#include "constants.h"                                               // program constants and structure definitions

//-------------------------------- Robot Definitions and Function Prototypes ------------------------------------------
int      initializeRobot(void);                                      // creates a TCP/IP connection between this program and the robot.
int      sendRobotCommand(const char *);                             // sends a command remotely to the SCARA robot
void     closeRobot(void);                                           // closes the TCP/IP connection to the robot

//----------------------------- Function Prototypes -------------------------------------------------------------------
void     waitForEnterKey(void);                                      // waits for the Enter key to be pressed
bool     flushInputBuffer(void);                                     // flushes any characters left in the standard input buffer
void     endProgram(void);                                           // ends the program from anywhere in the code
double   degToRad(double);                                           // returns angle in radians from input angle in degrees
double   radToDeg(double);                                           // returns angle in degrees from input angle in radians
double   mapAngle(double angRad);                                    // make sure inverseKinematic angled are mapped in range robot understands
bool     doAgain(void);                                              // asks user if they want to draw another shape
void     printRepeatedChar(char ch, int numRepeats);                 // prints an ascii character repeatedly
void     printTableHBorder(char chL, char chR, int tableWidth);      // prints table horizontal border

// prints joint angle and reachability data into table for a given [x,y] coordinate.
void     printPointData(int iPt, int NP, POINT2D pos, INVERSE_SOLUTION isol, int reachState);
void     printTableHeader(int tableWidth, const char *strTableTitle);// prints table header with centered title
int      getShapeChoice(void);                                       // gets the shape choice to draw from the user (LINE/ARC/BEZIER)
void     rotateRobotJoints(JOINT_ANGLES ja);                         // send command to robot to rotate the joints
void     drawShape(int shape);                                       // draw shape based on shape index

//----- MANDITORY FUNCTION PROTOTYPES -----
int      getNumPoints();

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  C program to draw various shapes with the SCARA robot
// ARGUMENTS:    none
// RETURN VALUE: an int that tells the O/S how the program ended.  0 = EXIT_SUCCESS = normal termination
int main(void) {
   int      iShape = -1;                           // shape index (LINE/ARC/QUADRATIC_BEZIER)

   if(!initializeRobot()) exit(0);                 // connect to the robot

   // draw shapes until user terminates
   do {
      iShape       = getShapeChoice();             // get users choice of shape to draw
      drawShape(iShape);                           // draw requested shape
   } while(doAgain());

   // set final position
   sendRobotCommand("PEN_UP\n");
   sendRobotCommand("HOME\n");
   endProgram();
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  draws a shape with a robot according to user specifications
// ARGUMENTS:    shape: index of the shape to draw (LINE/ARC/BEZIER)
// RETURN VALUE: none
void drawShape(int shape) {
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Asks the user if they want to draw another line
// ARGUMENTS:    none
// RETURN VALUE: true if they want to, false if not
bool doAgain(void) {
   return false;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  prints a table header with centered title (includes top/bottom borders)
// ARGUMENTS:    tableWdith:     width of table included left/right borders
//               strTableTitle:  title string
// RETURN VALUE: none
void printTableHeader(int tableWidth, const char *strTableTitle) {
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  prints data for one point into table (may be multiple lines).  prints bottom border but not top
// ARGUMENTS:    iPt:      The zero-based point index of the line/arc/quadratic bezier
//               NP:          The number of points of the line/arc/quadratic bezier, including endpoints
//               pos:         The x,y coordinates of the point
//               isol:        contains corresponding Joint angles (in degrees) for both arm + the reachability of both
//               reachState:  The combined reach state data (for contextual printing)
// RETURN VALUE: none
void printPointData(int iPt, int NP, POINT2D pos, INVERSE_SOLUTION isol, int reachState) {
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Prints a horiztonal border for a table
// ARGUMENTS:    chL/chR: left/right border characters
//               tableWidth: with of table including borders
// RETURN VALUE: none
void printTableHBorder(char chL, char chR, int tableWidth) {
   printf("%c", chL);
   printRepeatedChar(HL, tableWidth - 2);
   printf("%c\n", chR);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Prints a character to the console repeatedly
// ARGUMENTS:    ch: character to repeat
//               numRepeats: number of repeats
// RETURN VALUE: none
void printRepeatedChar(char ch, int numRepeats) {
   int i = -1; // loop counter

   for(i = 0; i < numRepeats; i++) {
      printf("%c", ch);
   }
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Maps an angle in radians into a an equivalent angle in degrees that follows the angle ranges
//               defined in the robot (-180 <= theta <= +180)
// ARGUMENTS:    ang: the angle in radians
// RETURN VALUE: the mapped angle in radians
double mapAngle(double theta) {
   theta                       = fmod(theta, 2.0 * PI);  // put in range -2*PI <= ang <= +2*PI

   // map into range -PI <= ang <= +PI
   if(theta > PI)       theta -= 2.0 * PI;
   else if(theta < -PI) theta += 2.0 * PI;

   return radToDeg(theta);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Returns angle in degrees from input angle in radian
// ARGUMENTS:    angDeg:  angle in degrees
// RETURN VALUE: angle in radians
double degToRad(double angDeg) {
   return (PI / 180.0) * angDeg;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Returns angle in radians from input angle in degrees
// ARGUMENTS:    angRad:  angle in radians
// RETURN VALUE: angle in degrees
double radToDeg(double angRad) {
   return (180.0 / PI) * angRad;
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  This function flushes the input buffer to avoid scanf issues
// ARGUMENTS:    none
// RETURN VALUE: false if nothing or only '\n' in stdin. true if extra keystrokes precede the '\n'.
//               Good for detecting left over garbage from scanf_s in the input buffer
bool flushInputBuffer(void) {
   int ch; // temp character variable
   bool bHasGarbage = false;

   // exit loop when all characters are flushed
   while((ch = getchar()) != '\n' && ch != EOF) {
      if(!bHasGarbage) bHasGarbage = true;
   }
   return bHasGarbage;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Waits for user to press enter.  flushes stdin if keystrokes precede enter
// ARGUMENTS:    none
// RETURN VALUE: none
void waitForEnterKey(void) {
   char buff[MAX_BUFF];
   fgets(buff, MAX_BUFF, stdin);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Ends program from anywhere in code.
// ARGUMENTS:    none
// RETURN VALUE: none
void endProgram(void) {
   printf("\nPress ENTER to end the program...\n");
   waitForEnterKey();
   exit(0);
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Move the SCARA Robot to specified joint angles
// ARGUMENTS:    theta1Deg, theta2Deg:  shoulder and elbow angles in degrees
// RETURN VALUE: none
void rotateRobotJoints(JOINT_ANGLES ja) {
   char strCommand[MAX_COMMAND];  // for storing robot command strings

   // compose the command string using sprintf_s
   sprintf_s(strCommand, MAX_COMMAND, "ROTATE_JOINT ANG1 %lf ANG2 %lf\n", ja.theta1Deg, ja.theta2Deg);

   // send the command to the robot
   sendRobotCommand(strCommand);
}

//-------------------------------------------------- Get Functions -----------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Asks the user what shape they want to draw
// ARGUMENTS:    none
// RETURN VALUE: shape choice index (see enum SHAPES for values)
int getShapeChoice(void) {
   char input;
   int bHasGarbage;
   while(true){
      printf("What shape would you like to draw [L for Line, A for Arc, Q for Quadratic Bezier]: ");
      scanf_s("%c", &input);
      bHasGarbage = flushInputBuffer();
      if(tolower(input) == 'l'){

         return LINE;
         break;
      }
      if(tolower(input) == 'a'){
         return ARC;
         break;
      }
      if(tolower(input) == 'q'){
         return QUADRATIC_BEZIER;
         break;
      }
      if(bHasGarbage >= 1){
         printf("please enter a single valid character\n");
      } else {
         printf("Bad input! Please enter L or A or Q (no leading/trailing characters)\n");
      }
   }

}
//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Asks the user for line points and the returns the input
// ARGUMENTS:    none
// RETURN VALUE: LINE_DATE struct
LINE_DATA getLineData(){
   int iret,bHasGarbage;
   //this is the bs comments were invented for
   LINE_DATA LINE_DATA;
   POINT2D pA,pB;
   while(true){
      printf("Enter the line end point coordinates xA,yA,xB,yB (comma seperated): ");
      iret = scanf_s("%lf,%lf,%lf,%lf",&pA.x,&pA.y,&pB.x,&pB.y);
      bHasGarbage = flushInputBuffer();
      if (iret == 4){
         printf("Got good end point values: xA, yA = [%.0lf,%.0lf] and xB, yB = [%.0lf,%.0lf].\n",pA.x,pA.y,pB.x,pB.y);
         break;
      } else {
         printf("Invalid input. Please try again. \n");
      }
   }
   LINE_DATA.pA = pA;
   LINE_DATA.pB = pB;
   LINE_DATA.NP = getNumPoints();
   return LINE_DATA;
}
//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Asks the user for the number of point to be used in a line
// ARGUMENTS:    none
// RETURN VALUE: NP number of point
int getNumPoints(){
   int bHasGarbage,NP;
   while(true)
   {
      printf("Enter the number of points on the line, NP: ");
      scanf_s("%d",&NP);
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
//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Asks the user for the points need to make a arc
// ARGUMENTS:    none
// RETURN VALUE: ARC_DATA
ARC_DATA getArcData(){
   ARC_DATA ARC_DATA;
   POINT2D pc;
   int NP, iret, bHasGarbage;
   double thetaStartDeg, thetaEndDeg, r;
   while(true){
      printf("Enter arc center point coordinates x,y (comma seperated):");
      iret=scanf_s("%lf,%lf",&pc.x,&pc.y);
      bHasGarbage=flushInputBuffer();
      if (iret == 2){
         if(bHasGarbage){
            printf("Got x and y values (%.*lf, %.*lf) but y value contains trailing non-numerical characters\n",PRECISION,pc.x,PRECISION,pc.y);
         } else {
            printf("Got good x and y values (%.*lf, %.*lf). Thanks!!\n",PRECISION,pc.x,PRECISION,pc.y);
            break;
         }
      } else if (iret == 1) {
         if(bHasGarbage){
            printf("Only got x value (%.*lf)!\n",PRECISION,pc.x);
            printf("x may have trailing garbage or y may have leading garbage or you forgot the comma.\n");
         } else {
            printf("to few arguments!\n");
         }
      } else {
         printf("Didnt get any usable data. x values contains leading non-numerical characters\n");
      }
   }
   while(true){
      printf("Enter arc angles in degrees thetaStart,thetaEnd (comma seperated):");
      iret=scanf_s("%lf,%lf",&thetaStartDeg,&thetaEndDeg);
      bHasGarbage=flushInputBuffer();
      if (iret == 2){
         if(bHasGarbage){
            printf("Got thetaStart and thetaEnd values (%.*lf, %.*lf) but y value contains trailing non-numerical characters\n",PRECISION,thetaStartDeg,PRECISION,thetaEndDeg);
         } else {
            printf("Got good thetaStart and thetaEnd values (%.*lf, %.*lf). Thanks!!\n",PRECISION,thetaStartDeg,PRECISION,thetaEndDeg);
            break;
         }
      } else if (iret == 1) {
         if(bHasGarbage){
            printf("Only got thetaStart value (%.*lf)!\n",PRECISION,thetaStartDeg);
            printf("thetaStart may have trailing garbage or thetaEnd may have leading garbage or you forgot the comma.\n");
         } else {
            printf("to few arguments!\n");
         }
      } else {
         printf("Didnt get any usable data. thetaStart values contains leading non-numerical characters\n");
      }
   }
   while(true)
   {
      printf("Enter the radius of the arc, r: ");
      scanf_s("%lf",&r);
      bHasGarbage = flushInputBuffer();
      if (bHasGarbage == 0)
      {
         printf("Got good value: NP = %*lf.\n",PRECISION,r);
         break;
      } else {
         printf("Invalid input. Please try again. \n");
      }
   }
   NP = getNumPoints;
}
//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Asks the user for the points need to make a quadratic bezier
// ARGUMENTS:    none
// RETURN VALUE: QUADRATIC_BEZIER_DATA
QUADRATIC_BEZIER_DATA getQuadraticBezierData(){
   QUADRATIC_BEZIER_DATA QUADRATIC_BEZIER_DATA;
   int iret, bHasGarbage, NP;
   POINT2D pA,pB,pC;
   while(true){
      printf("Enter 3 control point coordinates xA,yA,xB,yB,xC,yC (comma seperated): ");
      iret = scanf_s("%lf,%lf,%lf,%lf,%lf,%lf",&pA.x,&pA.y,&pB.x,&pB.y,&pC.x,&pC.y);
      bHasGarbage = flushInputBuffer();
      if (iret == 6){
         printf("Got good end point values: xA, yA = [%.0lf,%.0lf] xB, yB = [%.0lf,%.0lf] and xC, yC =[%.0lf,%.0lf].\n",pA.x,pA.y,pB.x,pB.y,pC.x,pC.y);
         break;
      } else {
         printf("Invalid input. Please try again. \n");
      }
   }
   NP = getNumPoints();
   return QUADRATIC_BEZIER_DATA;
}
