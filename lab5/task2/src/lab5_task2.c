/**********************************************************************************************************************
Course: MECH 7171 - Engineering Programming

Program: Lab 3: Structures

Details: Control the SCARA robot using various commands.  Draw lines and curves with various colors and line
         thicknesses. The line or curve will only be drawn if it can be drawn in its entirety using
         the left arm or right arm without switching arms at any point.  Components of the C program include
         variables (including pointers), formatted console output, error checked user input, branches, loops,
         functions, bitwise operations, structures.

Author(s): Walker Golembioski A01374098, Hayley Reeve-Hamilton A01021058

Declaration: We, Walker Golembioski and Hayley Reeve,  declare that the following program was written by us.

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
#include "lab5_task2.h"                         // your lab4 function prototypes
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
   switch (shape)
   {
   case ARC:
      drawArc();
      break;
   case LINE:
      drawLine();
      break;
   case QUADRATIC_BEZIER:
      drawQuadraticBezier();
      break;
   case CUBIC_BEZIER:
      drawCubicBezier();
      break;
   case TRIANGLE:
      drawTriangle();
      break;
   case RECTANGLE:
      drawRectangle();
      break;
   default:
      printf("AMONG US");
      break;
   }
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  draws a shape with a robot according to user specifications
// ARGUMENTS:    shape: index of the shape to draw (LINE/ARC/BEZIER)
// RETURN VALUE: none
void draw(int mshape, void *mshapeData){
   int shape = mshape;
   void *shapeData = mshapeData;
   LINE_DATA *pLine = NULL;
   ARC_DATA *pArc = NULL;
   QUADRATIC_BEZIER_DATA *pQB = NULL;
   CUBIC_BEZIER_DATA *pCB = NULL;
   TRIANGLE_DATA *pTri = NULL;
   RECTANGLE_DATA *pRect = NULL;
   TRACE_DATA tdat;
   char str[MAX_TITLE];
   tdat.bLeftCanDraw = true;
   tdat.bRightCanDraw = true;

   if (shape == LINE){
      pLine = (LINE_DATA*) shapeData;
      tdat.NP = (*pLine).NP;
      sprintf_s(str, sizeof(str), "LINE: PA = [%lg,%lg], PB = [%lg,%lg], NP = %zu",(*pLine).P[0].x,(*pLine).P[0].y,(*pLine).P[1].x,(*pLine).P[1].y,(*pLine).NP);
   }
   if (shape == ARC){
      pArc = (ARC_DATA*) shapeData;
      tdat.NP = (*pArc).NP;
      sprintf_s(str, sizeof(str), "ARC: P[2] = [%lg,%lg], R = %lg, Start Angle = %lg%c, End Angle = %lg%c, NP = %zu", (*pArc).pc.x,(*pArc).pc.y,(*pArc).r,(*pArc).thetaDeg[0],DEGREE_SYMBOL,(*pArc).thetaDeg[1],DEGREE_SYMBOL,(*pArc).NP);
   }
   if (shape == QUADRATIC_BEZIER){
      pQB = (QUADRATIC_BEZIER_DATA*) shapeData;
      tdat.NP = (*pQB).NP;
      sprintf_s(str, sizeof(str), "QUADRATIC BEZIER: pA = [%lg,%lg], pB = [%lg,%lg], pC = [%lg,%lg], NP = %zu", (*pQB).P[0].x,(*pQB).P[0].y,(*pQB).P[1].x,(*pQB).P[1].y,(*pQB).P[2].x,(*pQB).P[2].y,(*pQB).NP);
   }
   if (shape == CUBIC_BEZIER){
      pCB = (CUBIC_BEZIER_DATA*) shapeData;
      tdat.NP = (*pCB).NP;
      sprintf_s(str, sizeof(str), "CUBIC BEZIER: PA = [%lg,%lg], PB = [%lg,%lg], PC = [%lg,%lg], PD = [%lg,%lg], NP = %zu",(*pCB).P[0].x,(*pCB).P[0].y,(*pCB).P[1].x,(*pCB).P[1].y,(*pCB).P[2].x,(*pCB).P[2].y,(*pCB).P[3].x,(*pCB).P[3].y,(*pCB).NP);
   }
   if (shape == TRIANGLE){
      pTri = (TRIANGLE_DATA*) shapeData;
      tdat.NP = ((*pTri).NP[0] + (*pTri).NP[1] + (*pTri).NP[2] - 2);
      sprintf_s(str, sizeof(str), "TRIANGLE: PA = [%lg,%lg], PB = [%lg,%lg], PC = [%lg,%lg], NP1 = %zu, NP2 = %zu, NP3 = %zu",(*pTri).P[0].x,(*pTri).P[0].y,(*pTri).P[1].x,(*pTri).P[1].y,(*pTri).P[2].x,(*pTri).P[2].y,(*pTri).NP[0],(*pTri).NP[1],(*pTri).NP[2]);
   }
   if (shape == RECTANGLE){
      pRect = (RECTANGLE_DATA*) shapeData;
      tdat.NP =(2*(*pRect).NP[0] + 2*(*pRect).NP[1] - 3);
      sprintf_s(str, sizeof(str), "RECTANGLE: PA = [%lg,%lg], PB = [%lg,%lg], NP1 = %zu, NP2 = %zu",(*pRect).P[0].x,(*pRect).P[0].y,(*pRect).P[1].x,(*pRect).P[1].y,(*pRect).NP[0],(*pRect).NP[1]);
   }

   //make da array
   POINT_DATA *pointsData = malloc(sizeof(POINT_DATA)*tdat.NP);
   tdat.pointsData = pointsData;
   populateTraceData(shape, shapeData, &tdat);

   //print the things
   printTableHeader(TABLE_WIDTH,str);
   printTraceData(&tdat);

   //check if we can draw
   if(tdat.bLeftCanDraw || tdat.bRightCanDraw){
      getTraceAttributes(&tdat);
      setMotorSpeed(tdat.traceAttributes.motorSpeed);
      setPenColor(tdat.traceAttributes.penColor);
      robotTraceData(&tdat);
   } else {
      printf("shape no good :(");
   }

   //no memory leak :(
   free(pointsData);
}

void populateTraceData(int shapeType, const void *shapeData, TRACE_DATA *traceData){
   int i,n;
   if(shapeType == TRIANGLE){
      TRIANGLE_DATA *pTri = (TRIANGLE_DATA*) shapeData;
      for(n=0;n<=2;n++){
         LINE_DATA pPart;
         pPart.NP = (*pTri).NP[n];
         pPart.P[0] = (*pTri).P[n];
         if(n==2){
            pPart.P[1] = (*pTri).P[0];
         } else {
            pPart.P[1] = (*pTri).P[n+1];
         }
         if (n == 0){
            for(i=1;i<=pPart.NP;i++){
               int index;
               switch (n)
               {
               case 0:
                  index = i-1;
                  break;
               case 1:
                  index = i-1 + (*pTri).NP[0];
                  break;
               case 2:
                  index = i-1 + (*pTri).NP[0] + (*pTri).NP[1] - 1;
                  break;
               default:
                  break;
               }
               double t;
               if(n == 0 && i == 1){
                  t = 0;
               } else {
                  t = (i+1.0-1.0)/(pPart.NP+1.0-1.0);
               }
               (*traceData).pointsData[index].pt = getTooltipPos(LINE, &pPart, t);
               (*traceData).pointsData[index].isol.reachState = inverseKinematics((*traceData).pointsData[index].pt,&(*traceData).pointsData[index].isol);
               (*traceData).bLeftCanDraw &= (*traceData).pointsData[index].isol.bLeftCanReach;
               (*traceData).bRightCanDraw &= (*traceData).pointsData[index].isol.bRightCanReach;
            }
         } else {
            for(i=1;i<=pPart.NP-1;i++){
               int index;
               switch (n)
               {
               case 0:
                  index = i-1;
                  break;
               case 1:
                  index = i-1 + (*pTri).NP[0];
                  break;
               case 2:
                  index = i-1 + (*pTri).NP[0] + (*pTri).NP[1] - 1;
                  break;
               default:
                  break;
               }
               double t;
               if(n == 0 && i == 1){
                  t = 0;
               } else {
                  t = (i+1.0-1.0)/(pPart.NP-1.0);
               }
               (*traceData).pointsData[index].pt = getTooltipPos(LINE, &pPart, t);
               (*traceData).pointsData[index].isol.reachState = inverseKinematics((*traceData).pointsData[index].pt,&(*traceData).pointsData[index].isol);
               (*traceData).bLeftCanDraw &= (*traceData).pointsData[index].isol.bLeftCanReach;
               (*traceData).bRightCanDraw &= (*traceData).pointsData[index].isol.bRightCanReach;
            }
         }
      }
   } else if(shapeType == RECTANGLE){
      RECTANGLE_DATA *pRect = (RECTANGLE_DATA*) shapeData;
      POINT2D apoints[4];
      apoints[0] = (*pRect).P[0];
      apoints[1].x = (*pRect).P[0].x;
      apoints[1].y = (*pRect).P[1].y;
      apoints[2] = (*pRect).P[1];
      apoints[3].x = (*pRect).P[1].x;
      apoints[3].y = (*pRect).P[0].y;
      for(n=0;n<=3;n++){
         LINE_DATA pPart;
         pPart.NP = (*pRect).NP[n%2];
         pPart.P[0] = apoints[n];
         if(n==3){
            pPart.P[1] = apoints[0];
         } else {
            pPart.P[1] = apoints[n+1];
         }
         if (n==0){
            for(i=1;i<=pPart.NP;i++){
               int index;
               switch (n)
               {
               case 0:
                  index = i-1;
                  break;
               case 1:
                  index = i-1 + (*pRect).NP[0];
                  break;
               case 2:
                  index = i-1 + (*pRect).NP[0] + (*pRect).NP[1] - 1;
                  break;
               case 3:
                  index = i-1 + (2*(*pRect).NP[0]) + (*pRect).NP[1] - 2;
                  break;
               default:
                  break;
               }
               double t;
               if(n == 0 && i == 1){
                  t = 0;
               } else {
                  t = (i+1.0-1.0)/(pPart.NP-1.0+1.0);
               }
               (*traceData).pointsData[index].pt = getTooltipPos(LINE, &pPart, t);
               (*traceData).pointsData[index].isol.reachState = inverseKinematics((*traceData).pointsData[index].pt,&(*traceData).pointsData[index].isol);
               (*traceData).bLeftCanDraw &= (*traceData).pointsData[index].isol.bLeftCanReach;
               (*traceData).bRightCanDraw &= (*traceData).pointsData[index].isol.bRightCanReach;
            }
         } else {
            for(i=1;i<=pPart.NP-1;i++){
               int index;
               switch (n)
               {
               case 0:
                  index = i-1;
                  break;
               case 1:
                  index = i-1 + (*pRect).NP[0];
                  break;
               case 2:
                  index = i-1 + (*pRect).NP[0] + (*pRect).NP[1] - 1;
                  break;
               case 3:
                  index = i-1 + (2*(*pRect).NP[0]) + (*pRect).NP[1] - 2;
                  break;
               default:
                  break;
               }
               double t;
               if(n == 0 && i == 1){
                  t = 0;
               } else {
                  t = (i+1.0-1.0)/(pPart.NP-1.0);
               }
               (*traceData).pointsData[index].pt = getTooltipPos(LINE, &pPart, t);
               (*traceData).pointsData[index].isol.reachState = inverseKinematics((*traceData).pointsData[index].pt,&(*traceData).pointsData[index].isol);
               (*traceData).bLeftCanDraw &= (*traceData).pointsData[index].isol.bLeftCanReach;
               (*traceData).bRightCanDraw &= (*traceData).pointsData[index].isol.bRightCanReach;
            }
         }
      }
   } else {
      for(i=1;i<=(*traceData).NP;i++){
         double t = (i-1.0)/((*traceData).NP-1.0);
         (*traceData).pointsData[i-1].pt = getTooltipPos(shapeType, shapeData, t);
         (*traceData).pointsData[i-1].isol.reachState = inverseKinematics((*traceData).pointsData[i-1].pt,&(*traceData).pointsData[i-1].isol);
         (*traceData).bLeftCanDraw &= (*traceData).pointsData[i-1].isol.bLeftCanReach;
         (*traceData).bRightCanDraw &= (*traceData).pointsData[i-1].isol.bRightCanReach;
      }
   }

}
void printTraceData(const TRACE_DATA *traceData){
   int i;
   for(i=1;i<=(*traceData).NP;i++){
      double t = (i-1.0)/((*traceData).NP-1.0);
      printPointData(i-1,(*traceData).NP,(*traceData).pointsData[i-1].pt,(*traceData).pointsData[i-1].isol,(*traceData).pointsData[i-1].isol.reachState);
      if (i == (*traceData).NP){
         printTableHBorder(BL,BR,TABLE_WIDTH);
      } else {
         printTableHBorder(CL,CR,TABLE_WIDTH);
      }
   }
}
void getTotalAngleChanges(TRACE_DATA *traceData){
   double arm1R, arm2R, arm1L, arm2L;
   arm1R = (*traceData).pointsData[(*traceData).NP-1].isol.rightArm.theta1Deg - (*traceData).pointsData[0].isol.rightArm.theta1Deg;
   arm2R = (*traceData).pointsData[(*traceData).NP-1].isol.rightArm.theta2Deg - (*traceData).pointsData[0].isol.rightArm.theta2Deg;
   if(abs(arm1R) > abs(arm2R)){
      (*traceData).rightArmDeltaDeg = arm1R;
   } else {
      (*traceData).rightArmDeltaDeg = arm2R;
   }
   arm1L = (*traceData).pointsData[(*traceData).NP-1].isol.leftArm.theta1Deg - (*traceData).pointsData[0].isol.leftArm.theta1Deg;
   arm2L = (*traceData).pointsData[(*traceData).NP-1].isol.leftArm.theta2Deg - (*traceData).pointsData[0].isol.leftArm.theta2Deg;
   if(abs(arm1L) > abs(arm2L)){
      (*traceData).leftArmDeltaDeg = arm1L;
   } else {
      (*traceData).leftArmDeltaDeg = arm2L;
   }
}
void robotTraceData(const TRACE_DATA *traceData){
   int i;
   for(i=1;i<=(*traceData).NP;i++){
      //Move the robot with pen up to the starting position
      if(i==1){setPenPos(PEN_UP);}
      //Check for valid moves
      if((*traceData).bLeftCanDraw == true){
         rotateRobotJoints((*traceData).pointsData[i-1].isol.leftArm);
      } else if ((*traceData).bRightCanDraw == true){
         rotateRobotJoints((*traceData).pointsData[i-1].isol.rightArm);
      }
      if(i==1){setPenPos(PEN_DOWN);}
   }
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
      printf("What shape would you like to draw [L for Line, A for Arc, Q for Quadratic Bezier, C for Cubic Bezier, T for Triangle, R for Rectangle]: ");
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
      if(tolower(input) == 'c'){
         return CUBIC_BEZIER;
         break;
      }
      if(tolower(input) == 't'){
         return TRIANGLE;
         break;
      }
      if(tolower(input) == 'r'){
         return RECTANGLE;
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
   POINT2D P[2];
   while(true){
      printf("Enter the line end point coordinates xA,yA,xB,yB (comma seperated): ");
      iret = scanf_s("%lf,%lf,%lf,%lf",&P[0].x,&P[0].y,&P[1].x,&P[1].y);
      bHasGarbage = flushInputBuffer();
      if (iret == 4){
         printf("Got good end point values: xA, yA = [%.0lf,%.0lf] and xB, yB = [%.0lf,%.0lf].\n",P[0].x,P[0].y,P[1].x,P[1].y);
         break;
      } else {
         printf("Invalid input. Please try again. \n");
      }
   }
   LINE_DATA.P[0] = P[0];
   LINE_DATA.P[1] = P[1];
   printf("For the line ");LINE_DATA.NP = getNumPoints();
   return LINE_DATA;
}
//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Asks the user for the number of point to be used in a line
// ARGUMENTS:    none
// RETURN VALUE: NP number of point
size_t getNumPoints(){
   int bHasGarbage,NP;
   while(true)
   {
      printf("enter the number of points, NP: ");
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
   double thetaDeg[2], r;
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
   while(true)
   {
      printf("Enter the radius of the arc, r: ");
      scanf_s("%lf",&r);
      bHasGarbage = flushInputBuffer();
      if (bHasGarbage == 0)
      {
         printf("Got good value: R = %*lf.\n",PRECISION,r);
         break;
      } else {
         printf("Invalid input. Please try again. \n");
      }
   }
   while(true){
      printf("Enter arc angles in degrees thetaStart,thetaEnd (comma seperated):");
      iret=scanf_s("%lf,%lf",&thetaDeg[0],&thetaDeg[1]);
      bHasGarbage=flushInputBuffer();
      if (iret == 2){
         if(bHasGarbage){
            printf("Got thetaStart and thetaEnd values (%.*lf, %.*lf) but y value contains trailing non-numerical characters\n",PRECISION,thetaDeg[0],PRECISION,thetaDeg[1]);
         } else {
            printf("Got good thetaStart and thetaEnd values (%.*lf, %.*lf). Thanks!!\n",PRECISION,thetaDeg[0],PRECISION,thetaDeg[1]);
            break;
         }
      } else if (iret == 1) {
         if(bHasGarbage){
            printf("Only got thetaStart value (%.*lf)!\n",PRECISION,thetaDeg[0]);
            printf("thetaStart may have trailing garbage or thetaEnd may have leading garbage or you forgot the comma.\n");
         } else {
            printf("to few arguments!\n");
         }
      } else {
         printf("Didnt get any usable data. thetaStart values contains leading non-numerical characters\n");
      }
   }
   printf("For the arc ");NP = getNumPoints();
   ARC_DATA.pc = pc;
   ARC_DATA.thetaDeg[0] = thetaDeg[0];
   ARC_DATA.thetaDeg[1] = thetaDeg[1];
   ARC_DATA.r = r;
   ARC_DATA.NP = NP;
   return ARC_DATA;
}
//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Asks the user for the points need to make a quadratic bezier
// ARGUMENTS:    none
// RETURN VALUE: QUADRATIC_BEZIER_DATA
QUADRATIC_BEZIER_DATA getQuadraticBezierData(){
   QUADRATIC_BEZIER_DATA QUADRATIC_BEZIER_DATA;
   int iret, bHasGarbage, NP;
   POINT2D P[3];
   while(true){
      printf("Enter 3 control point coordinates xA,yA,xB,yB,xC,yC (comma seperated): ");
      iret = scanf_s("%lf,%lf,%lf,%lf,%lf,%lf",&P[0].x,&P[0].y,&P[1].x,&P[1].y,&P[2].x,&P[2].y);
      bHasGarbage = flushInputBuffer();
      if (iret == 6){
         printf("Got good end point values: xA, yA = [%.0lf,%.0lf] xB, yB = [%.0lf,%.0lf] and xC, yC =[%.0lf,%.0lf].\n",P[0].x,P[0].y,P[1].x,P[1].y,P[2].x,P[2].y);
         break;
      } else {
         printf("Invalid input. Please try again. \n");
      }
   }
   printf("For the quadratic bezier "); NP = getNumPoints();
   QUADRATIC_BEZIER_DATA.NP = NP;
   QUADRATIC_BEZIER_DATA.P[0] = P[0];
   QUADRATIC_BEZIER_DATA.P[1] = P[1];
   QUADRATIC_BEZIER_DATA.P[2] = P[2];
   return QUADRATIC_BEZIER_DATA;
}

CUBIC_BEZIER_DATA getCubicBezierData(){
   CUBIC_BEZIER_DATA CUBIC_BEZIER_DATA;
   int iret, bHasGarbage, NP;
   POINT2D P[4];

   while(true){
      printf("Enter 4 control point coordinates xA,yA,xB,yB,xC,yC,xD,yD (comma seperated): ");
      iret = scanf_s("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",&P[0].x,&P[0].y,&P[1].x,&P[1].y,&P[2].x,&P[2].y,&P[3].x,&P[3].y);
      bHasGarbage = flushInputBuffer();
      if (iret == 8){
         printf("Got good end point values: xA, yA = [%.0lf,%.0lf] xB, yB = [%.0lf,%.0lf] xC, yC = [%.0lf,%.0lf] and xD, yD [%.0lf,%.0lf].\n",P[0].x,P[0].y,P[1].x,P[1].y,P[2].x,P[2].y,&P[3].x,&P[3].y);
         break;
      } else {
         printf("Invalid input. Please try again. \n");
      }
   }
   NP = getNumPoints();
   CUBIC_BEZIER_DATA.NP = NP;
   CUBIC_BEZIER_DATA.P[0] = P[0];
   CUBIC_BEZIER_DATA.P[1] = P[1];
   CUBIC_BEZIER_DATA.P[2] = P[2];
   CUBIC_BEZIER_DATA.P[3] = P[3];
   return CUBIC_BEZIER_DATA;
}

TRIANGLE_DATA getTriangleData(){
   TRIANGLE_DATA TRIANGLE_DATA;
   int iret, bHasGarbage, NP[3];
   POINT2D P[3];
   while(true){
      printf("Enter the 3 point of the triangle xA,yA,xB,yB,xC,yC (comma seperated): ");
      iret = scanf_s("%lf,%lf,%lf,%lf,%lf,%lf",&P[0].x,&P[0].y,&P[1].x,&P[1].y,&P[2].x,&P[2].y);
      bHasGarbage = flushInputBuffer();
      if (iret == 6){
         printf("Got good point values: xA, yA = [%.0lf,%.0lf] xB, yB = [%.0lf,%.0lf] and xC, yC =[%.0lf,%.0lf].\n",P[0].x,P[0].y,P[1].x,P[1].y,P[2].x,P[2].y);
         break;
      } else {
         printf("Invalid input. Please try again. \n");
      }
   }
   printf("For side 1 "); NP[0] = getNumPoints();
   printf("For side 2 "); NP[1] = getNumPoints();
   printf("For side 3 "); NP[2] = getNumPoints();
   TRIANGLE_DATA.NP[0] = NP[0];
   TRIANGLE_DATA.NP[1] = NP[1];
   TRIANGLE_DATA.NP[2] = NP[2];
   TRIANGLE_DATA.P[0] = P[0];
   TRIANGLE_DATA.P[1] = P[1];
   TRIANGLE_DATA.P[2] = P[2];
   return TRIANGLE_DATA;
}

RECTANGLE_DATA getRectangleData(){
   int iret,bHasGarbage;
   RECTANGLE_DATA RECTANGLE_DATA;
   POINT2D P[2];
   while(true){
      printf("Enter the bottom left point and the top right point xA,yA,xB,yB (comma seperated): ");
      iret = scanf_s("%lf,%lf,%lf,%lf",&P[0].x,&P[0].y,&P[1].x,&P[1].y);
      bHasGarbage = flushInputBuffer();
      if (iret == 4){
         printf("Got good point values: xA, yA = [%.0lf,%.0lf] and xB, yB = [%.0lf,%.0lf].\n",P[0].x,P[0].y,P[1].x,P[1].y);
         break;
      } else {
         printf("Invalid input. Please try again. \n");
      }
   }
   RECTANGLE_DATA.P[0] = P[0];
   RECTANGLE_DATA.P[1] = P[1];
   printf("For the right and left edges ");RECTANGLE_DATA.NP[0] = getNumPoints();
   printf("For the top and bottom edges ");RECTANGLE_DATA.NP[1] = getNumPoints();
   return RECTANGLE_DATA;
}

void getTraceAttributes(TRACE_DATA *traceData){
   TRACE_ATTRIBUTES TRACE_ATTRIBUTES;
   char colour;
   char thickness;
   int bHasGarbage;
   TRACE_ATTRIBUTES.penColor = NO_COLOR;
   TRACE_ATTRIBUTES.motorSpeed = 69;
   //jet jolour
   printf("Choose the letter corresponding to the colour you want:\n\n");
   printf("   A=Aqua   B=Black   G=Green   H=HotPink   O=Orange\n");
   printf("   R=Red    W=White   N=Navy    Y=Yellow    P=Purple\n\n");
   while(true){
      printf("Choice:");
      scanf_s("%c", &colour);
      bHasGarbage = flushInputBuffer();
      if(bHasGarbage == 0){
         colour = tolower(colour);
         switch (colour)
         {
         case 'a':
            TRACE_ATTRIBUTES.penColor = AQUA;
            break;
         case 'b':
            TRACE_ATTRIBUTES.penColor = BLACK;
            break;
         case 'g':
            TRACE_ATTRIBUTES.penColor = GREEN;
            break;
         case 'h':
            TRACE_ATTRIBUTES.penColor = HOTPINK;
            break;
         case 'o':
            TRACE_ATTRIBUTES.penColor = ORANGE;
            break;
         case 'r':
            TRACE_ATTRIBUTES.penColor = RED;
            break;
         case 'w':
            TRACE_ATTRIBUTES.penColor = WHITE;
            break;
         case 'n':
            TRACE_ATTRIBUTES.penColor = NAVY;
            break;
         case 'y':
            TRACE_ATTRIBUTES.penColor = YELLOW;
            break;
         case 'p':
            TRACE_ATTRIBUTES.penColor = PURPLE;
            break;
         default:
            printf("Invalid Input\n");
            break;
         }
         if (TRACE_ATTRIBUTES.penColor.r != -1){
            break;
         }
      } else {
         printf("Please enter a single character (no leading/trailing spaces).\n");
      }
   }
   // met mortor meed
   while(true){
      printf("Choose the line thickness [T = Thin, M = Medium, K = thicK]:");
      scanf_s("%c", &thickness);
      bHasGarbage = flushInputBuffer();
      if(bHasGarbage == 0){
         thickness = tolower(thickness);
         switch (thickness)
         {
         case 't':
            TRACE_ATTRIBUTES.motorSpeed = MOTOR_SPEED_HIGH;
            break;
         case 'm':
            TRACE_ATTRIBUTES.motorSpeed = MOTOR_SPEED_MEDIUM;
            break;
         case 'k':
            TRACE_ATTRIBUTES.motorSpeed = MOTOR_SPEED_LOW;
            break;
         default:
            printf("Invalid Input\n");
            break;
         }
         if(TRACE_ATTRIBUTES.motorSpeed != 69){
            break;
         }
      } else {
         printf("Please enter a single character (no leading/trailing spaces).\n");
      }
   }
   (*traceData).traceAttributes = TRACE_ATTRIBUTES;
}
//-------------------------------------------------- Math Functions -----------------------------------------------------------
POINT2D getTooltipPos(int shape, void *shapeData, double t){
   LINE_DATA *pLine = NULL;
   ARC_DATA *pArc = NULL;
   QUADRATIC_BEZIER_DATA *pQB = NULL;
   CUBIC_BEZIER_DATA *pCB = NULL;
   TRIANGLE_DATA *pTri = NULL;
   RECTANGLE_DATA *pRect = NULL;
   POINT2D toolTipPos;

   if (shape == LINE){
      pLine = (LINE_DATA*) shapeData;
      toolTipPos.x = ((1-t)*(*pLine).P[0].x) + (((*pLine).P[1].x)*t);
      toolTipPos.y = ((1-t)*(*pLine).P[0].y) + (((*pLine).P[1].y)*t);
      return toolTipPos;
   }
   if (shape == ARC){
      pArc = (ARC_DATA*) shapeData;
      double thetai = (((*pArc).thetaDeg[0])*(1-t)) + (((*pArc).thetaDeg[1])*t);
      toolTipPos.x = (*pArc).pc.x + ((*pArc).r * cos(degToRad(thetai)));
      toolTipPos.y = (*pArc).pc.y + ((*pArc).r * sin(degToRad(thetai)));
      return toolTipPos;
   }
   if (shape == QUADRATIC_BEZIER){
      pQB = (QUADRATIC_BEZIER_DATA*) shapeData;
      toolTipPos.x = (1-t)*(1-t)*(*pQB).P[0].x + 2*(1-t)*t*(*pQB).P[1].x + t*t*(*pQB).P[2].x;
      toolTipPos.y = (1-t)*(1-t)*(*pQB).P[0].y + 2*(1-t)*t*(*pQB).P[1].y + t*t*(*pQB).P[2].y;
      return toolTipPos;
   }
   if (shape == CUBIC_BEZIER){
      pCB = (CUBIC_BEZIER_DATA*) shapeData;
      toolTipPos.x = (1-t)*(1-t)*(1-t)*(*pCB).P[0].x + 3*(1-t)*(1-t)*t*(*pCB).P[1].x + 3*(1-t)*t*t*(*pCB).P[2].x + t*t*t*(*pCB).P[3].x;
      toolTipPos.y = (1-t)*(1-t)*(1-t)*(*pCB).P[0].y + 3*(1-t)*(1-t)*t*(*pCB).P[1].y + 3*(1-t)*t*t*(*pCB).P[2].y + t*t*t*(*pCB).P[3].y;
      return toolTipPos;
   }
}
//-------------------------------------------------- Draw Functions -----------------------------------------------------------
void drawArc(){
   ARC_DATA data = getArcData();
   draw(ARC, &data);
}
void drawLine(){
   LINE_DATA data = getLineData();
   draw(LINE,&data);
}
void drawQuadraticBezier(){
   QUADRATIC_BEZIER_DATA data = getQuadraticBezierData();
   draw(QUADRATIC_BEZIER,&data);
}
void drawCubicBezier(){
   CUBIC_BEZIER_DATA data = getCubicBezierData();
   draw(CUBIC_BEZIER,&data);
}
void drawTriangle(){
   TRIANGLE_DATA data = getTriangleData();
   draw(TRIANGLE,&data);
}
void drawRectangle(){
   RECTANGLE_DATA data = getRectangleData();
   draw(RECTANGLE,&data);
}
//-------------------------------------------------- Set Functions ------------------------------------------------------------
void setPenPos(int penPos){
   if (penPos == PEN_UP){
      sendRobotCommand("PEN_UP\n");
   }
   if (penPos == PEN_DOWN){
      sendRobotCommand("PEN_DOWN\n");
   }
}
void setPenColor(RGB color){
   char str[MAX_COMMAND];
   sprintf_s(str,sizeof(str),"PEN_COLOR %d %d %d\n",color.r, color.g, color.b);
   sendRobotCommand(str);
}
void setMotorSpeed(int motor){
   if(motor == MOTOR_SPEED_HIGH){
      sendRobotCommand("MOTOR_SPEED HIGH\n");
   }
   if(motor == MOTOR_SPEED_MEDIUM){
      sendRobotCommand("MOTOR_SPEED MEDIUM\n");
   }
   if(motor == MOTOR_SPEED_LOW){
      sendRobotCommand("MOTOR_SPEED LOW\n");
   }
}

// :)