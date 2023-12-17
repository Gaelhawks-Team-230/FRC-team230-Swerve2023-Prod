#pragma once

#define LOOPTIME (0.02)

#define ONE_SEC ((int)(1.0 / LOOPTIME))
#define HALF_SEC ((int)(0.5 * ONE_SEC))
#define QUART_SEC ((int)(0.25 * ONE_SEC))

// * Resets all CTRE Falcons and encoders to factory state
// #define CTRE_FACTORY_RESET
// * Turns off all swerve motors to calibrate positions
// #define SWERVE_CALIBRATION_MODE

// * Enables print statements for drivetrain system id
// #define SWERVE_SYS_ID

// absolute encoder constants
#define DMAX (1024.0 / 1025.0)
#define DMIN (1.0 / 1025.0)

// DIO assignments
// typedef enum
// {
// } digitals;

// (FC) - Swerve scale factors
#define XY_SFACTOR (200.0)
#define XY_SFACTOR_LOW (100.0)
#define R_SFACTOR (200.0)
#define R_SFACTOR_LOW (100.0)

// use numbers below for slowing drivetrain
// #define XY_SFACTOR (50.0)
// #define R_SFACTOR (50.0)

// (FC) - Flight controller
#define FC_X_DEADBAND (0.1)
#define FC_Y_DEADBAND (0.1)
#define FC_R_DEADBAND (0.1)
#define FC_X_SHAPING (0.6)
#define FC_Y_SHAPING (0.6)
#define FC_R_SHAPING (1.0)

#define SIGN(x) (((x) > 0) ? (1) : (-1))

// CAN device IDs
typedef enum
{
    PDP_MODULE = 0,
    // Swerve
    // Module 1
    FRONT_RIGHT_DRIVE = 1,
    FRONT_RIGHT_STEER = 5,
    FRONT_RIGHT_ABSOLUTE_ENCODER = 9,
    // Module 2
    BACK_RIGHT_DRIVE = 2,
    BACK_RIGHT_STEER = 6,
    BACK_RIGHT_ABSOLUTE_ENCODER = 10,
    // Module 3
    BACK_LEFT_DRIVE = 3,
    BACK_LEFT_STEER = 7,
    BACK_LEFT_ABSOLUTE_ENCODER = 11,
    // Module 4
    FRONT_LEFT_DRIVE = 4,
    FRONT_LEFT_STEER = 8,
    FRONT_LEFT_ABSOLUTE_ENCODER = 12,

} can;
// typedef enum
// {
// } analog;

// typedef enum
// {
// } pdp;

// typedef enum
// {
// } pcm;

// typedef enum
// {
// } pwm;

typedef enum
{
    X_AXIS = 1,
    Y_AXIS = 5,
    Z_AXIS = 0

} flightctrl_axis;

// typedef enum
// {
// } gamepad_axis;

typedef enum
{
    GYRO_BUTTON_SWITCH = 1,
    FIELD_ROBOT_SWITCH = 2,
    RESET_GYRO_BUTTON = 3,
    // VISION_LEFT_BUTTON = 6,
    // VISION_RIGHT_BUTTON = 7,
    GRID_ALIGN_BUTTON_1 = 6,
    GRID_ALIGN_BUTTON_2 = 7,
    WRIST_BIAS_INCREASE = 8,
    WRIST_BIAS_DECREASE = 9,
    BEEP_FOREWARD_BUTTON = 10,
    BEEP_BACKWARD_BUTTON = 11,
    BEEP_RIGHT_BUTTON = 12,
    BEEP_LEFT_BUTTON = 13,
    VISION_DEBUG_BUTTON = 14,
    VISION_CENTER_BUTTON = 15,
    VISION_PIPELINE_UP = 16,




} flightctrl_buttons;

typedef enum
{
    STOW_POS_BUTTON = 1,
    TOP_ROW_BUTTON = 2,
    MIDDLE_ROW_BUTTON = 3,
    BOTTOM_ROW_BUTTON = 4,
    COLLECT_CUBE_BUTTON = 5,
    COLLECT_CONE_BUTTON = 6,
    EJECT_CUBE_BUTTON = 7,
    EJECT_CONE_BUTTON = 8,
    // VISION_LEFT_BUTTON = 9,
    // VISION_CENTER_BUTTON = 10,
    SUB_OPEN_BUTTON = 9,
    SUB_BIAS_CLOSE_BUTTON = 10,


} gamepad_buttons;

typedef enum
{
    STOW_POS = 0,
    HIGH_POS = 1,
    MID_POS = 2,
    LOW_POS = 3,
    PICKUP_POS = 4,
    SUB_POS = 5,
    REARPICKUP_POS = 6,
} Position;

typedef enum
{
    NONE = 0,
    CONE = 1,
    CUBE = 2
} Gamepiece;

typedef struct
{
    double x;
    double y;
    double z;
    double rx;
    double ry;
    double rz;
} Pose;

typedef struct
{
    Pose pose;
    double fID;
    double ta;
} Target;

typedef enum
{
    ROBOT_FRAME = 0,
    FIELD_FRAME = 1,
    FIELD_CENTER = 2

} VisionFrame;
typedef enum
{
    BEST_TARGET = 0,
    LOCK_TARGET = 1
} TargetFilter;
typedef enum
{
    DRIVER_CAM = 0,
    APRIL_TAG_CAM = 1
} CameraMode;

#define GRID_ALIGN_ANGLE (0.0)
#define LOADING_ZONE_ANGLE (180)
#define GYRO_ANGLE_KP (2.0)