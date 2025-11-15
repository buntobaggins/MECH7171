#pragma once

#ifndef LAB4_CONSTANTS_H
#define LAB4_CONSTANTS_H

#define _USE_MATH_DEFINES

#include <stdlib.h>                       // standard functions and constant
#include <stdio.h>                        // i/o functions
#include <math.h>                         // math functions
#include <string.h>                       // string functions
#include <ctype.h>                        // character functions
#include <stdbool.h>                      // bool definitions
#include <stdarg.h>                       // for variable argument functions
#include <share.h>                        // for file sharing

//---------------------------- Program Constants ------------------------------------------------------------
const double         PI                    = M_PI;       // the one and only

// !!!! hardcoding these because used as initializes for const variables !!!!
const double         LIMIT_TOL             = 1.0e-10;     // crazy precision!!!
const double         POSITION_TOLERANCE    = 1.0e-4;      // check if same point
const double         ANGLE_TOLERANCE       = 1.0e-4;      // check if arm angle

const double         L1                    = 350.0;      // length of the inner arm
const double         L2                    = 250.0;      // length of the outer arm
#define              LMAX                    (600.0)     // max L -> maximum reach of robot
#define              LMIN                    (100.0)     // min L -> minimum reach of robot
const double         ABS_THETA1_DEG_MAX    = 150.0;      // maximum magnitude of shoulder angle in degrees
const double         ABS_THETA2_DEG_MAX    = 170.0;      // maximum magnitude of elbow angle in degrees


const int            PRECISION             = 4;          // for printing values to console
const int            FIELD_WIDTH           = 9;          // for printing values to console

const int            LEFT_MARGIN           = 2;          // number of spaces between left border and the first character printed
const int            TABLE_WIDTH           = 110;        // Table width for points data (width includes left/right borders)

// TABLE BORDER SYMBOLS
const unsigned char  HL                    = 196;        // horizontal border line
const unsigned char  VL                    = 179;        // vertical border line
const unsigned char  TL                    = 218;        // top left border symbol
const unsigned char  TC                    = 194;        // top center border symbol
const unsigned char  TR                    = 191;        // top right border symbol
const unsigned char  CL                    = 195;        // left center border symbol
const unsigned char  CC                    = 197;        // center center border symbol (cross)
const unsigned char  CR                    = 180;        // right center border symbol
const unsigned char  BL                    = 192;        // bottom left border symbol
const unsigned char  BC                    = 193;        // bottom center border symbol
const unsigned char  BR                    = 217;        // bottom right border symbol

const unsigned char  DEGREE_SYMBOL         = 248;        // the degree symbol
const unsigned char  THETA_SYMBOL          = 233;        // theta symbol
const unsigned char  ERROR_SYMBOL_LEFT     = '>';        // error symbol (left side)
const unsigned char  ERROR_SYMBOL_RIGHT    = '<';        // error symbol (right side)
const int            NUM_ERROR_SYMBOLS     = 7;          // number of leading/trailing error symbols to print

// constants to indicate reach errors
// constants to indicate reach errors
const int            L_EXCEEDS_MIN         = 1 << 0;     // (1)  L < LMIN
const int            L_EXCEEDS_MAX         = 1 << 1;     // (2)  L > LMAX
const int            THETA1L_EXCEEDS_MAX   = 1 << 2;     // (4)  |theta1LDeg| > ABS_THETA1_DEG_MAX
const int            THETA2L_EXCEEDS_MAX   = 1 << 3;     // (8)  |theta2LDeg| > ABS_THETA2_DEG_MAX
const int            THETA1R_EXCEEDS_MAX   = 1 << 4;     // (16) |theta1RDeg| > ABS_THETA1_DEG_MAX
const int            THETA2R_EXCEEDS_MAX   = 1 << 5;     // (32) |theta2RDeg| > ABS_THETA2_DEG_MAX

enum FILE_ERRORS {
   BLANK_LINE                              = -1,         // blank line encountered in input file
   UNKNOWN_COMMAND                         = -2,         // unknown command keyword encountered in input file
   MOTOR_SPEED_ERROR                       = -3,         // unknown motor speed parameter
   NP_ERROR                                =  0          // bad NP value

};

// various static array sizes
enum ARRAY_SIZES { MAX_COMMAND = 256, MAX_TITLE = 256, MAX_BUFF = 1024, MAX_FILENAME = 512, MAX_LINE = 1024 };
// motor speed
enum MOTOR_SPEED { MOTOR_SPEED_LOW, MOTOR_SPEED_MEDIUM, MOTOR_SPEED_HIGH };
// left arm or right arm configuration
enum ARM { LEFT, RIGHT };
// list of all command indexes
enum COMMAND_INDEX {
   LINE, ARC, QUADRATIC_BEZIER, CUBIC_BEZIER, TRIANGLE, RECTANGLE,
   ROTATE_JOINTS, MOTOR_SPEED, CYCLE_PEN_COLORS, PEN_UP, PEN_DOWN, PEN_COLOR, CLEAR_TRACE,
   CLEAR_REMOTE_COMMAND_LOG, CLEAR_POSITION_LOG, MESSAGE, SHUTDOWN_SIMULATION, END, HOME,
   MOVE_TO, ROTATE, TRANSLATE, SCALE, RESET_TRANSFORMATION_MATRIX
};

//-------------------------------- Structure Definitions ----------------------------------------------------
// structure to map command keyword string to a command index
#ifndef __cplusplus
typedef struct COMMAND {
   const int index;
   const char *strCommand;
} COMMAND;
#endif

// RGB color
typedef struct RGB {
   int r, g, b;  // ranges are 0-255
} RGB;

// SCARA tooltip coordinates
typedef struct TOOL_POSITION {
   double x, y;
} TOOL_POSITION, POINT2D;

// SCARA joint angles (degrees)
typedef struct JOINT_ANGLES {
   double theta1Deg, theta2Deg;
} JOINT_ANGLES;

// color and thickness of line/curve/arc etc to be drawn
typedef struct TRACE_ATTRIBUTES {
   RGB penColor;                    // trace color
   int motorSpeed;                  // related to trace thickness
   char strDescription[MAX_TITLE];  // short description of the trace (shape)
} TRACE_ATTRIBUTES;

// data for the current state of the robot
typedef struct SCARA_STATE {
   RGB            penColor;
   int            penPos;
   int            motorSpeed;
   JOINT_ANGLES   jointAngles;
} SCARA_STATE;

// inverse kinematics solution data
typedef struct INVERSE_SOLUTION {
   JOINT_ANGLES   leftArm, rightArm;               // left and right arm joint angles (in degrees)
   int            reachState;                      // compound value for limit checks
   bool           bLeftCanReach, bRightCanReach;   // true if robot arm configuration can reach, false if not
} INVERSE_SOLUTION;

// combination of x,y position and inverse solution values
typedef struct POINT_DATA {
   POINT2D pt;             // x,y position
   INVERSE_SOLUTION isol;  // inverse solution values (angles, reachability)
} POINT_DATA;

// ALL info needed to draw a shape with the robot
typedef struct TRACE_DATA {
   TRACE_ATTRIBUTES  traceAttributes;                    // shape color, motor speed
   POINT_DATA       *pointsData;                         // data for all points on the shape
   size_t            NP;                                 /// number of points on the shape
   bool              bLeftCanDraw, bRightCanDraw;        /// arm drawability for shape
   double            leftArmDeltaDeg, rightArmDeltaDeg;  /// arm angle deltas (to judge drawing time for each arm)
} TRACE_DATA;

// input data needed to draw an arc
typedef struct ARC_DATA {
   POINT2D pc;                         // center of arc
   double r;                           // arc radius
   double thetaDeg[2];                 // start and end point angles (relative to arc center)
   size_t NP;                          // number of points on arc (including endpoints)
   TRACE_ATTRIBUTES traceAttributes;   // motor speed and color for the line
} ARC_DATA;

// input data needed to draw an quadratic curve
typedef struct QUADRATIC_BEZIER_DATA {
   POINT2D P[3];                       // control point coordinates
   size_t NP;                          // number of points on bezier trace (including endpoints)
   TRACE_ATTRIBUTES traceAttributes;   // thickness and color of line
} QUADRATIC_BEZIER_DATA;

// input data needed to draw an quadratic curve
typedef struct CUBIC_BEZIER_DATA {
   POINT2D P[4];                       // control point coordinates
   size_t NP;                          // number of points on bezier trace (including endpoints)
   TRACE_ATTRIBUTES traceAttributes;   // thickness and color of line
} CUBIC_BEZIER_DATA;

// input data needed to draw lines
typedef struct LINE_DATA {
   POINT2D P[2];                       // endpoints
   size_t  NP;                         // number of points on the line
   TRACE_ATTRIBUTES traceAttributes;   // thickness and color of line
} LINE_DATA;

// input data needed to draw triangles
typedef struct TRIANGLE_DATA {
   POINT2D P[3];                       // vertices
   size_t  NP[3];                      // number of points on each line segment
   TRACE_ATTRIBUTES traceAttributes;   // thickness and color of line
} TRIANGLE_DATA;

// input data needed to draw rectangles.  Note that the data file only includes the bottom left and the
// top right vertices.  Others
typedef struct RECTANGLE_DATA {
   POINT2D P[2];                       // bottom left and top right vertices
   size_t  NP[2];                      // number of points on left/right and bottom/top edges
   TRACE_ATTRIBUTES traceAttributes;   // thickness and color of line
} RECTANGLE_DATA;

//---------------------------- Structure Global Constants ---------------------------------------------------------------
extern const RGB AQUA;
extern const RGB BLACK;
extern const RGB GREEN;
extern const RGB HOTPINK;
extern const RGB NAVY;
extern const RGB ORANGE;
extern const RGB PURPLE;
extern const RGB RED;
extern const RGB WHITE;
extern const RGB YELLOW;
extern const RGB PEN_COLOR_ERROR;
extern const RGB NO_COLOR;

#endif