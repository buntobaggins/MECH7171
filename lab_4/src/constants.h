#pragma once

#define _USE_MATH_DEFINES

#include <math.h>

//---------------------------- Program Constants ------------------------------------------------------------
const double         PI                    = M_PI;       // the one and only

// !!!! hardcoding these because used as initializes for const variables !!!!
#define              LMAX                    (600.0)     // max L -> maximum reach of robot
#define              LMIN                    (100.0)     // min L -> minimum reach of robot
const double         L_CHECK_TOLERENCE     = 1.0e-9;     // for L == LMIN or L == LMAX check
const double         L1                    = 350.0;      // length of the inner arm
const double         L2                    = 250.0;      // length of the outer arm
const double         ABS_THETA1_DEG_MAX    = 150.0;      // maximum magnitude of shoulder angle in degrees
const double         ABS_THETA2_DEG_MAX    = 170.0;      // maximum magnitude of elbow angle in degrees


const int            PRECISION             = 4;          // for printing values to console
const int            FIELD_WIDTH           = 9;          // for printing values to console

const int            TABLE_WIDTH           = 90;         // Table width for points data (width includes left/right borders)
const int            LEFT_MARGIN           = 2;          // number of spaces between left border and the first character printed

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
const int            L_EXCEEDS_MIN         = 1 << 0;     // (1)  L < LMIN
const int            L_EXCEEDS_MAX         = 1 << 1;     // (2)  L > LMAX
const int            THETA1L_EXCEEDS_MAX   = 1 << 2;     // (4)  |theta1LDeg| > ABS_THETA1_DEG_MAX
const int            THETA2L_EXCEEDS_MAX   = 1 << 3;     // (8)  |theta2LDeg| > ABS_THETA2_DEG_MAX
const int            THETA1R_EXCEEDS_MAX   = 1 << 4;     // (16) |theta1RDeg| > ABS_THETA1_DEG_MAX
const int            THETA2R_EXCEEDS_MAX   = 1 << 5;     // (32) |theta2RDeg| > ABS_THETA2_DEG_MAX

// SCARA strings, table titles
enum ARRAY_SIZES  {
   MAX_COMMAND                             = 256,
   MAX_TITLE                               = 86,
   MAX_BUFF                                = 1024
};
// motor speed indexes
enum MOTOR_SPEED  {
   MOTOR_SPEED_LOW,
   MOTOR_SPEED_MEDIUM,
   MOTOR_SPEED_HIGH
};
// left arm or right arm configuration
enum ARM          {
   LEFT,
   RIGHT
};
// indexes of the SCARA pen position
enum PEN_POS      {
   PEN_UP,
   PEN_DOWN
};
// shapes to draw
enum SHAPE        {
   LINE,
   ARC,
   QUADRATIC_BEZIER
};

//-------------------------------- Structure Definitions ----------------------------------------------------
// RGB color
typedef struct RGB {
   int            r, g, b;                      // ranges are 0-255
} RGB;

// SCARA tooltip coordinates
typedef struct TOOL_POSITION {
   double         x, y;
} TOOL_POSITION, POINT2D;

// SCARA joint angles (degrees)
typedef struct JOINT_ANGLES {
   double         theta1Deg, theta2Deg;
} JOINT_ANGLES;

// color and thickness of line/curve/arc etc to be drawn
typedef struct TRACE_ATTRIBUTES {
   RGB            penColor;                     // line color
   int            motorSpeed;                   // related to line thickness
} TRACE_ATTRIBUTES;

// inverse kinematics solution data
typedef struct INVERSE_SOLUTION {
   JOINT_ANGLES   leftArm, rightArm;            // left and right arm joint angles (in degrees)
   bool           bLeftCanReach, bRightCanReach;// true if robot can reach, false if not.  Left and right arm configurations
} INVERSE_SOLUTION;

// data needed to draw a line
typedef struct LINE_DATA {
   POINT2D        pA, pB;                       // start and end point coordinates
   int            NP;                           // number of points on line (including endpoints)
} LINE_DATA;

// data needed to draw an arc
typedef struct ARC_DATA {
   POINT2D        pc;                           // center of arc
   double         r;                            // arc radius
   double         thetaStartDeg, thetaEndDeg;   // start and end point angles (relative to arc center)
   int            NP;                           // number of points on arc (including endpoints)
} ARC_DATA;

// data needed to draw an quadratic bezier curve
typedef struct QUADRATIC_BEZIER_DATA {
   POINT2D        pA, pB, pC;                   // start, mid, end control point coordinates
   int            NP;                           // number of points on bezier (including endpoints)
} QUADRATIC_BEZIER_DATA;


//---------------------------- Structure Constants ---------------------------------------------------------------
// colors
const RGB            AQUA                  = {0,255,255};
const RGB            BLACK                 = {0,0,0};
const RGB            GREEN                 = {0,255,0};
const RGB            HOTPINK               = {255,20,147};
const RGB            NAVY                  = {0,0,255};
const RGB            ORANGE                = {255,165,0};
const RGB            PURPLE                = {155,0,155};
const RGB            RED                   = {255,0,0};
const RGB            WHITE                 = {255,255,255};
const RGB            YELLOW                = {255,255,0};
const RGB            NO_COLOR              = {-1,-1,-1};

// home joint angle and tool position
const JOINT_ANGLES   HOME_ANGLES           = {0.0, 0.0};
const TOOL_POSITION  HOME_POSITION         = {LMAX, 0.0};