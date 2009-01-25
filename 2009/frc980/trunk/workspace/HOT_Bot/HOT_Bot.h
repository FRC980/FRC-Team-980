#ifndef HOT_BOT_H
#define HOT_BOT_H

// uncomment to turn on printf debugging
#define _PRINTFDEBUG
#ifdef _PRINTFDEBUG
#define PRINTFDEBUG(x) printf x
#else
#define PRINTFDEBUG(x)
#endif

// math constants
#define PI			3.14159

// vehicle constants (mm)
#define WHEEL_DIAMETER	159.    // mm
#define WHEEL_RADIUS	(WHEEL_DIAMETER/2.)
#define WHEELBASE		505.    // mm
#define TRACK			559.    // mm
#define BOT_CENTER_X	343.    // mm
#define BOT_CENTER_Y	648.    // mm
#define ENCODER_COUNTS_PER_REV 400.
#define MM_PER_COUNT	(PI*WHEEL_DIAMETER/ENCODER_COUNTS_PER_REV)

// gyro sensitivity
#define GYRO_SENSITIVITY	0.0125  // 2006-2008 KOP, volts/degree/second
//#define GYRO_SENSITIVITY      0.716   // 2006-2008 KOP, volts/radian/second
//#define GYRO_SENSITIVITY      0.0125  // ADXRS150, volts/degree/second
//#define GYRO_SENSITIVITY      0.716   // ADXRS150, volts/radian/second
//#define GYRO_SENSITIVITY      0.005   // ADXRS300, volts/degree/second
//#define GYRO_SENSITIVITY      0.286   // ADXRS300, volts/radian/second

// arm position in volts
#define ARM_POSITION_STOWED	0.175   // folded and stowed position
#define ARM_POSITION_GROUND	0.800   // ground position
#define ARM_POSITION_BOTTOM	2.670   // bottom-level scoring position
#define ARM_POSITION_MIDDLE	3.500   // mid-level scoring position
#define ARM_POSITION_TOP	4.800   // top-level scoring position
#define ARM_POSITION_STEP	0.1

#define MIN_PARTICLE_TO_IMAGE_PERCENT 0.01      // target is too small
#define MAX_PARTICLE_TO_IMAGE_PERCENT 10.0      // target is too close

#endif
