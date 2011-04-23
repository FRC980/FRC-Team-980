//-- Axes

//Left thumbstick
#define XB_AXIS_LEFT_X          1
#define XB_AXIS_LEFT_Y          2

//Trigger (left=positive; right=negative)
#define XB_AXIS_TRIGGER         3

//Right thumbstick
#define XB_AXIS_RIGHT_X         4
#define XB_AXIS_RIGHT_Y         5

//D-pad(only reads -1,0, or 1)
#define XB_AXIS_DPAD_X          6
#define XB_AXIS_DPAD_Y          7


//-- Buttons
#define XB_BUTTON_A             1
#define XB_BUTTON_B             2
#define XB_BUTTON_X             3
#define XB_BUTTON_Y             4

#define XB_BUTTON_BUMPER_LEFT   5
#define XB_BUTTON_BUMPER_RIGHT  6

#define XB_BUTTON_BACK          7
#define XB_BUTTON_START         8


//-- Aliases - drive controller
#define DRIVE_SLOW_MODE         XB_BUTTON_BUMPER_LEFT
#define DRIVE_FAST_MODE         XB_BUTTON_BUMPER_RIGHT
#define DRIVE_LED_NONE          XB_BUTTON_A
#define DRIVE_LED_TRIANGLE      XB_BUTTON_B
#define DRIVE_LED_CIRCLE        XB_BUTTON_Y
#define DRIVE_LED_SQUARE        XB_BUTTON_X

//-- Aliases- set variables from controller
//            to diable, make all values different and out of range
#define DRIVE_VAR_CLEAR         XB_BUTTON_B +50
#define DRIVE_VAR_ZERO          XB_BUTTON_X +50
#define DRIVE_VAR_ONE           XB_BUTTON_Y +50
#define DRIVE_VAR_SUBMIT        XB_BUTTON_A +50


//-- Aliases - arm controller
#define ARM_POSITION_GROUND         8
#define ARM_POSITION_SIDE_LOW       7
#define ARM_POSITION_SIDE_MIDDLE    5
#define ARM_POSITION_SIDE_HIGH      3
#define ARM_POSITION_CENTER_LOW     6
#define ARM_POSITION_CENTER_MIDDLE  4
#define ARM_POSITION_CENTER_HIGH    2
#define ARM_POSITION_CARRY          1

#define ARM_DEPLOY                  11
#define ARM_CLAW                    9


//-- Aliases - debug controller
#define DEBUG_PRINT_LINETRACKER XB_BUTTON_Y
#define DEBUG_PRINT_ARM_STATE   XB_BUTTON_X

#define DEBUG_AUTON_INIT        XB_BUTTON_A +20
#define DEBUG_RUN_AUTON_MODE    XB_BUTTON_B +20

// This buttons is on the joystick handle
#define JS_TRIGGER  	1       /*!< joystick trigger */

// These buttons are all on the "top" of the joystick handle
#define JS_TOP_BOTTOM	2       /*!< joystick top, bottom button */
#define JS_TOP_CENTER	3       /*!< joystick top, center button */
#define JS_TOP_LEFT 	4       /*!< joystick top, left button */
#define JS_TOP_RIGHT	5       /*!< joystick top, right button */

// These buttons are all on the joystick base
#define JS_LEFT_TOP 	6       /*!< left side base, away from user */
#define JS_LEFT_BOTTOM 	7       /*!< left side base, close to user */
#define JS_BASE_LEFT	8       /*!< center of base, left side */
#define JS_BASE_RIGHT	9       /*!< center of base, right side */
#define JS_RIGHT_BOTTOM	10      /*!< right side base, close to user */
#define JS_RIGHT_TOP	11      /*!< right side base, away from user */
