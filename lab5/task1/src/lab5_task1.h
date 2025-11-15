#pragma once

#ifndef MECH7171_LAB4_H
#define MECH7171_LAB4_H

#include "constants.h"

// for echoing all console ouput to a log file
#define printf    DSprintf
#define scanf_s   DSscanf_s

//-------------------------------- Robot Definitions and Function Prototypes ------------------------------------------
extern int         initializeRobot(void);          // creates a TCP/IP connection between this program and the robot.
extern int         sendRobotCommand(const char *); // sends a command remotely to the SCARA robot
extern void        closeRobot(void);               // closes the TCP/IP connection to the robot
extern SCARA_STATE getRobotState(void);            // gets the current state of the robot (joint angles, pen position, pen color)

//---------------------------- Global Constants -----------------------------------------------------------------------
const RGB AQUA              = {  0, 255, 255};
const RGB BLACK             = {  0,   0,   0};
const RGB GREEN             = {  0, 255,   0};
const RGB HOTPINK           = {255,  20, 147};
const RGB NAVY              = {  0,   0, 255};
const RGB ORANGE            = {255, 165,   0};
const RGB PURPLE            = {155,   0, 155};
const RGB RED               = {255,   0,   0};
const RGB WHITE             = {255, 255, 255};
const RGB YELLOW            = {255, 255,   0};
const RGB PEN_COLOR_ERROR   = { -1,  -1,  -1};
const RGB NO_COLOR          = { -1,  -1,  -1};

//!!!!!!!!!!!!!!!!!!!!!! GLOBAL FILE HANDLE (FOR LOG FILE) !!!!!!!!!!!!!!!!!!!!!
FILE *m_flog = NULL;
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//----------------------------- Your Function Prototypes --------------------------------------------------------------
void                    setMotorSpeed(int motorSpeed);                  // sets the robot motor speed
void                    setPenPos(int pos);                             // sets the robot pen position (up or down)

int                     getShapeChoice(void);                           // gets the shape choice to draw from the user (LINE/ARC/BEZIER)
void                    drawShape(int shape);                           // draw shape based on shape index



#endif