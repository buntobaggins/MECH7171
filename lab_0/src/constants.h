//---------------------------- Program Constants ------------------------------------------------------------
enum PEN_POS {          // indexes of the SCARA pen position
   PEN_UP,
   PEN_DOWN
};
enum ARM {              // left arm or right arm configuration
   LEFT,
   RIGHT
};
enum MOTOR_SPEED {      // motor speed
   MOTOR_SPEED_LOW,
   MOTOR_SPEED_MEDIUM,
   MOTOR_SPEED_HIGH
};

//-------------------------------- Structure Definitions ----------------------------------------------------
// RGB color
typedef struct RGB {
   int r, g, b, pad;  // ranges are 0-255
} RGB;

// SCARA tooltip coordinates
typedef struct TOOL_POSITION {
   double x, y;
} TOOL_POSITION;

// SCARA joint angles (degrees)
typedef struct JOINT_ANGLES {
   double theta1Deg, theta2Deg;
} JOINT_ANGLES;

typedef struct SCARA_STATE {
   RGB penColor;
   int penPos;
   int motorSpeed;
   JOINT_ANGLES jointAngles;

} SCARA_STATE;

//---------------------------- Structure Constants ---------------------------------------------------------------
const RGB RED      = {255,0,0};
const RGB GREEN    = {0,255,0};
const RGB BLUE     = {0,0,255};
const RGB BLACK    = {0,0,0};
