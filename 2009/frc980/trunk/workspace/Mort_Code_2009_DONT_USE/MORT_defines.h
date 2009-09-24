#ifndef _MORT__DEFINES_H_
#define _MORT__DEFINES_H_

#include "MORT_includes.h"

/***** Math Constants******/
static const float e = 2.718;

/***** MOTOR and Sensor Assignments******/

static const UINT32 RIGHT_FRONT_MOTOR = 2;
static const UINT32 LEFT_BACK_MOTOR = 1;
static const UINT32 RIGHT_BACK_MOTOR = 4;
static const UINT32 LEFT_FRONT_MOTOR = 3;
static const UINT32 TOWER_MOTOR = 6;
static const UINT32 ROLLER_MOTOR = 5;

static const UINT32 LEFT_JOYSTICK = 2;
static const UINT32 RIGHT_JOYSTICK = 3;
static const UINT32 TOWER_JOYSTICK = 4;

static const float RampUpSpeed = 100;   //15000;  //90000;
static const float BrakingConstant = e;
//static const float BrakeConstant = -1 * 9998.0;

static const float RampConstant = 2;
static const int LEFT_JOY = 1;
static const int RIGHT_JOY = 2;

/******Solenoid out*******/
static const UINT32 SOLENOID = 1;

/*****Digital Input*****/
static const UINT32 PUMP_SENSOR = 1;
static const UINT32 AUTO_SWITCH = 3;

/*****Relay******/
static const UINT32 COMPRESSOR_RELAY = 1;

/******Autonomous Constants*****/

static const int G_RED_MIN = 72;
static const int G_RED_MAX = 140;
static const int G_GREEN_MIN = 160;
static const int G_GREEN_MAX = 219;
static const int G_BLUE_MIN = 80;
static const int G_BLUE_MAX = 146;

static const int R_RED_MIN = 236;
static const int R_RED_MAX = 248;
static const int R_GREEN_MIN = 55;
static const int R_GREEN_MAX = 159;
static const int R_BLUE_MIN = 102;
static const int R_BLUE_MAX = 134;

static const int PARTICLE_PERCENT_MIN = 1;
static const int PARTICLE_PERCENT_MAX = 40;
static const float MIN_OFFSET = 0.7;

static const float LEFT = -1.0;
static const float RIGHT = 1.0;

static const int RD = 0;
static const int GR = 1;

static const int LEFT_SIDE = 1;
static const int RIGHT_SIDE = 2;

/***** FUNCTION PROTOTYPES *****/
float __(int);
void ___(float, float);
void _(void);
void ____(float, float);

void ________(float, float, int, int);
void _________(int);
int  __________();

void ColPrintReport(ColorReport *);
void ParPrintReport(ParticleAnalysisReport *);
void ThreshPrintReport(TrackingThreshold *);

float fabs(float);              // was _____ (5)
float _______(float min, float max, float val); // limit
float ______(float, float);     // max_abs(a,b)

#endif
