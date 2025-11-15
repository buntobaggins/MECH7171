#pragma once

#ifndef LAB4_GIVEN_FUNCTIONS_H
#define LAB4_GIVEN_FUNCTIONS_H

#include "constants.h"  // program constants and structure definitions

//!!!!!!!!!!!!!!!!!!!!!! GLOBAL FILE HANDLE (FOR LOG FILE) !!!!!!!!!!!!!!!!!!!!!
extern FILE *m_flog;
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//-------------------------------- Robot Definitions and Function Prototypes ------------------------------------------
int      initializeRobot(void);                                         // creates a TCP/IP connection between this program and the robot.
int      sendRobotCommand(const char *);                                // sends a command remotely to the SCARA robot
void     closeRobot(void);                                              // closes the TCP/IP connection to the robot

void     makeStringUpperCase(char *string);                             // converts all alphabet characters in the string to upper case
void     removeNewline(char *string);                                   // removes '\n' from end of the string
void     endProgram(const char *strMessage);                            // prints a message and ends the program from anywhere in the code
void     waitForEnterKey(void);                                         // waits for the Enter key to be pressed
bool     flushInputBuffer(void);                                        // flushes any characters left in the standard input buffer
double   degToRad(double);                                              // returns angle in radians from input angle in degrees
double   radToDeg(double);                                              // returns angle in degrees from input angle in radians
double   mapAngle(double angRad);                                       // make sure inverseKinematic angled are mapped in range robot understands
bool     doAgain(void);                                                 // asks user if they want to draw another shape
int      DSprintf(char *fmt, ...);                                      // prints to console AND file at the same time
int      DSscanf_s(char *fmt, ...);                                     // reads from console and echos to file
bool     openLogFile(void);                                             // opens the log file to capture console output
void     rotateRobotJoints(JOINT_ANGLES ja);                            // send command to robot to rotate the joints
void     printRepeatedChar(char ch, int reps);                          // prints an ascii character repeatedly
void     printGuide(int tableWidth);                                    // prints a guide for a table to judge column alignment
void     printTableHeader(int tableWidth, const char *strTableTitle);   // prints table header with centered title
void     printTableHBorder(char chL, char chR, int tableWidth);         // prints table horizontal border
int      inverseKinematics(TOOL_POSITION pos, INVERSE_SOLUTION *pSol);  // inverse kinematics calcs based on tool position

// prints table header with centered title on the first line and desciption of the table contents on the second line
void     printTableHeader2(int tableWidth, const char *strTableTitle, const char *strDesciption);

// prints joint angle and reachability data into table for a given [x,y] coordinate.
void     printPointData(size_t iPt, size_t NP, POINT2D pos, INVERSE_SOLUTION isol, int reachState);

void     transform(POINT2D *ppt, const double TM[3][3]);                // applies the transformation matrix a coordinate point
void     resetTransformationMatrix(double TM[3][3]);                    // resets to unit matrix
void     transformMatrixMultiply(double TM[][3], const double M[][3]);  // adds scaling or rotation or translation to TM


#endif