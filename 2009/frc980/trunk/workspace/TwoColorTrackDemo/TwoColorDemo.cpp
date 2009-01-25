/********************************************************************************
*  Project   		: FIRST Motor Controller
*  File Name  		: TwoColorDemo.cpp        
*  Contributors 	: ELF
*  Creation Date 	: Jan 3, 2009
*  Revision History	: Source code & revision history maintained at sourceforge.WPI.edu   
*  File Description	: Demo program showing color tracking using servos
*/
/*----------------------------------------------------------------------------*/
/*        Copyright (c) FIRST 2008.  All Rights Reserved.                     */
/*  Open Source Software - may be modified and shared by FRC teams. The code  */
/*  must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib. */
/*----------------------------------------------------------------------------*/

#include <iostream.h>
#include "math.h"

#include "AxisCamera.h"
#include "BaeUtilities.h"
#include "FrcError.h"
#include "TrackAPI.h"
#include "Target.h"
#include "WPILib.h"

// To locally enable debug printing: set the debugFlag to a 1, to disable set to 0
static int TwoColorDemo_debugFlag = 0;
#define DPRINTF if(TwoColorDemo_debugFlag)dprintf

#define PI 3.14159265358979

// for 160x120, 50 pixels = .38 %
#define MIN_PARTICLE_TO_IMAGE_PERCENT 0.25      // target is too small
#define MAX_PARTICLE_TO_IMAGE_PERCENT 20.0      // target is too close

/** Simple test to see if the color is taking up too much of the image */
int tooClose(ParticleAnalysisReport * par)
{
    if (par->particleToImagePercent > MAX_PARTICLE_TO_IMAGE_PERCENT)
        return 1;
    return 0;
}

/** Simple test to see if the color is large enough */
int bigEnough(ParticleAnalysisReport * par)
{
    if (par->particleToImagePercent < MIN_PARTICLE_TO_IMAGE_PERCENT)
        return 0;
    return 1;
}

/**
 * This is a demo program showing the use of the color tracking API to track two colors.
 * It uses the SimpleRobot class as a base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls. Autonomous mode tracks color assuming camera is 
 * mounted on a gimbal with two servos.
 */
class TwoColorDemo:public SimpleRobot
{
    RobotDrive *myRobot;        // robot drive system
    Joystick *rightStick;       // joystick 1 (arcade stick or right tank stick)
    Joystick *leftStick;        // joystick 2 (tank left stick)
    DriverStation *ds;          // driver station object
    Servo *horizontalServo;     // first servo object
    Servo *verticalServo;       // second servo object
    float horizontalDestination;
    float verticalDestination;
    float horizontalPosition, verticalPosition;
    float horizontalServoPosition, verticalServoPosition;
    float servoDeadband;        // percentage servo delta to trigger move
    int  framesPerSecond;       // number of camera frames to get per second
    float panControl;           // to slow down pan
    float panPosition;
    double sinStart;
    static double panStartTime;
    TrackingThreshold td1, td2; // color thresholds

    ParticleAnalysisReport par; // particle analysis report

    enum                        // Driver Station jumpers to control program operation
    { ARCADE_MODE = 1,          // Tank/Arcade jumper is on DS Input 1 (Jumper present is arcade)
        ENABLE_AUTONOMOUS = 2,  // Autonomous/Teleop jumper is on DS Input 2 (Jumper present is autonomous)
    } jumpers;

  public:
        /**
	 * Constructor for this robot subclass.
	 * Create an instance of a RobotDrive with left and right motors plugged into PWM
	 * ports 1 and 2 on the first digital module. The two servos are PWM ports 3 and 4.
	 * Tested with camera settings White Balance: Flurorescent1, Brightness 40, Exposure Auto
	 */
         TwoColorDemo(void)
    {
        ds = DriverStation::GetInstance();
        myRobot = new RobotDrive(1, 2, 0.5);    // robot will use PWM 1-2 for drive motors
        rightStick = new Joystick(1);   // create the joysticks
        leftStick = new Joystick(2);
        // remember to use jumpers on the sidecar for the Servo PWMs
        horizontalServo = new Servo(9); // create horizontal servo on PWM 9
        verticalServo = new Servo(10);  // create vertical servo on PWM 10
        servoDeadband = 0.01;   // move if > this amount 
        framesPerSecond = 15;   // number of camera frames to get per second
        sinStart = 0.0;         // control where to start the sine wave for pan
        memset(&par, 0, sizeof(par));   // initialize particle analysis report

        /* image data for tracking - override default parameters if needed */
        /* recommend making PINK the first color because GREEN is more 
         * subsceptible to hue variations due to lighting type so may
         * result in false positives */
        // PINK
        sprintf(td1.name, "PINK");
        td1.hue.minValue = 220;
        td1.hue.maxValue = 255;
        td1.saturation.minValue = 75;
        td1.saturation.maxValue = 255;
        td1.luminance.minValue = 85;
        td1.luminance.maxValue = 255;
        // GREEN
        sprintf(td2.name, "GREEN");
        td2.hue.minValue = 55;
        td2.hue.maxValue = 125;
        td2.saturation.minValue = 58;
        td2.saturation.maxValue = 255;
        td2.luminance.minValue = 92;
        td2.luminance.maxValue = 255;

        /* set up debug output: 
         * DEBUG_OFF, DEBUG_MOSTLY_OFF, DEBUG_SCREEN_ONLY, DEBUG_FILE_ONLY, DEBUG_SCREEN_AND_FILE 
         */
        SetDebugFlag(DEBUG_SCREEN_ONLY);

        /* start the CameraTask  */
        if (StartCameraTask(framesPerSecond, 0, k320x240, ROT_0) == -1)
        {
            DPRINTF(LOG_ERROR,
                    "Failed to spawn camera task; exiting. Error code %s",
                    GetVisionErrorText(GetLastVisionError()));
        }
        /* allow writing to vxWorks target */
        Priv_SetWriteFileAllowed(1);

        /* stop the watchdog if debugging  */
        GetWatchdog().SetExpiration(0.5);
        GetWatchdog().SetEnabled(false);
    }

        /**
	 * Set servo positions (0.0 to 1.0) translated from normalized values (-1.0 to 1.0). 
	 * 
	 * @param normalizedHorizontal Pan Position from -1.0 to 1.0.
	 * @param normalizedVertical Tilt Position from -1.0 to 1.0.
	 */
    void setServoPositions(float normalizedHorizontal,
                           float normalizedVertical)
    {

        float servoH = NormalizeToRange(normalizedHorizontal);
        float servoV = NormalizeToRange(normalizedVertical);

        float currentH = horizontalServo->Get();
        float currentV = verticalServo->Get();

        /* make sure the movement isn't too small */
        if (fabs(servoH - currentH) > servoDeadband)
        {
            horizontalServo->Set(servoH);
            /* save new normalized horizontal position */
            horizontalPosition = RangeToNormalized(servoH, 1);
        }
        if (fabs(servoV - currentV) > servoDeadband)
        {
            verticalServo->Set(servoV);
            verticalPosition = RangeToNormalized(servoV, 1);
        }
    }

        /**
	 * Adjust servo positions (0.0 to 1.0) translated from normalized values (-1.0 to 1.0). 
	 * 
	 * @param normalizedHorizontal Pan adjustment from -1.0 to 1.0.
	 * @param normalizedVertical Tilt adjustment from -1.0 to 1.0.
	 */
    void adjustServoPositions(float normDeltaHorizontal,
                              float normDeltaVertical)
    {

        /* adjust for the fact that servo overshoots based on image input */
        normDeltaHorizontal /= 8.0;
        normDeltaVertical /= 4.0;

        /* compute horizontal goal */
        float currentH = horizontalServo->Get();
        float normCurrentH = RangeToNormalized(currentH, 1);
        float normDestH = normCurrentH + normDeltaHorizontal;
        /* narrow range keep servo from going too far */
        if (normDestH > 1.0)
            normDestH = 1.0;
        if (normDestH < -1.0)
            normDestH = -1.0;
        /* convert input to servo range */
        float servoH = NormalizeToRange(normDestH);

        /* compute vertical goal */
        float currentV = verticalServo->Get();
        float normCurrentV = RangeToNormalized(currentV, 1);
        float normDestV = normCurrentV + normDeltaVertical;
        if (normDestV > 1.0)
            normDestV = 1.0;
        if (normDestV < -1.0)
            normDestV = -1.0;
        /* convert input to servo range */
        float servoV = NormalizeToRange(normDestV, 0.2, 0.8);

        /* make sure the movement isn't too small */
        if (fabs(currentH - servoH) > servoDeadband)
        {
            horizontalServo->Set(servoH);
            /* save new normalized horizontal position */
            horizontalPosition = RangeToNormalized(servoH, 1);
        }
        if (fabs(currentV - servoV) > servoDeadband)
        {
            verticalServo->Set(servoV);
            verticalPosition = RangeToNormalized(servoV, 1);
        }
    }

        /**
	 * Adjusts the servo gimbal based on the color tracked.
	 * Driving the robot or operating an arm based on color input from gimbal-mounted 
	 * camera is currently left as an exercise for the teams.
	 */
    void Autonomous(void)
    {
        char funcName[] = "Autonomous";
        DPRINTF(LOG_DEBUG, "Autonomous");

        DPRINTF(LOG_DEBUG, "SERVO - looking for COLOR %s ABOVE %s",
                td2.name, td1.name);

        // initialize position and destination variables
        // position settings range from -1 to 1
        // setServoPositions is a wrapper that handles the conversion to range for servo 
        horizontalDestination = 0.0;    // final destination range -1.0 to +1.0
        verticalDestination = 0.0;

        // initialize pan variables
        // incremental tasking toward dest (-1.0 to 1.0)
        float incrementH, incrementV;
        // pan needs a 1-up number for each call
        int  panIncrement = 0;

        // current position range -1.0 to +1.0
        horizontalPosition = RangeToNormalized(horizontalServo->Get(), 1);
        verticalPosition = RangeToNormalized(verticalServo->Get(), 1);

        // set servos to start at center position
        setServoPositions(horizontalDestination, verticalDestination);

        // for controlling loop execution time 
        float loopTime = 0.1;
        //float loopTime = 0.05;                                                                                        
        double currentTime = GetTime();
        double lastTime = currentTime;

        // search variables 
        bool foundColor = 0;
        double savedImageTimestamp = 0.0;
        bool staleImage = false;

        while (IsAutonomous())
        {
            //GetWatchdog().Feed();         // turn watchdog off while debugging    

            // calculate gimbal position based on colors found 
            if (FindTwoColors(td1, td2, ABOVE, &par))
            {
                //PrintReport(&par);
                foundColor = true;
                // reset pan            
                panIncrement = 0;
                if (par.imageTimestamp == savedImageTimestamp)
                {
                    // This image has been processed already, 
                    // so don't do anything for this loop 
                    staleImage = true;
                    DPRINTF(LOG_DEBUG, "STALE IMAGE");

                }
                else
                {
                    // The target was recognized
                    // save the timestamp
                    staleImage = false;
                    savedImageTimestamp = par.imageTimestamp;
                    DPRINTF(LOG_DEBUG, "image timetamp: %lf",
                            savedImageTimestamp);

                    // Here is where your game-specific code goes
                    // when you recognize the target

                    // get center of target 

                    // TODO: use par.average_mass_x_normalized and
                    // par.average_mass_y_normalized after updated WPILib is distributed
                    horizontalDestination = par.center_mass_x_normalized;
                    verticalDestination = par.center_mass_y_normalized;
                }
            }
            else
            {                   // need to pan 
                foundColor = false;
            }

            if (foundColor && !staleImage)
            {
                /* Move the servo a bit each loop toward the destination.
                 * Alternative ways to task servos are to move immediately vs.
                 * incrementally toward the final destination. Incremental method
                 * reduces the need for calibration of the servo movement while
                 * moving toward the target.
                 */
                incrementH = horizontalDestination - horizontalPosition;
                // you may need to reverse this based on your vertical servo installation
                //incrementV = verticalPosition - verticalDestination;
                incrementV = verticalDestination - verticalPosition;
                adjustServoPositions(incrementH, incrementV);

                ShowActivity("** %s & %s found: Servo: x: %f  y: %f ** ",
                             td1.name, td2.name, horizontalDestination,
                             verticalDestination);

            }
            else
            {                   //if (!staleImage) {  // new image, but didn't find two colors

                // adjust sine wave for panning based on last movement direction
                if (horizontalDestination > 0.0)
                {
                    sinStart = PI / 2.0;
                }
                else
                {
                    sinStart = -PI / 2.0;
                }

                /* pan to find color after a short wait to settle servos
                 * panning must start directly after panInit or timing will be off */
                if (panIncrement == 3)
                {
                    panInit(8.0);       // number of seconds for a pan
                }
                else if (panIncrement > 3)
                {
                    panForTarget(horizontalServo, sinStart);

                    /* Vertical action: In case the vertical servo is pointed off center,
                     * center the vertical after several loops searching */
                    if (panIncrement == 20)
                    {
                        verticalServo->Set(0.5);
                    }
                }
                panIncrement++;

                ShowActivity
                    ("** %s and %s not found                                    ",
                     td1.name, td2.name);
            }                   // end if found color

            // sleep to keep loop at constant rate
            // this helps keep pan consistant
            // elapsed time can vary significantly due to debug printout
            currentTime = GetTime();
            lastTime = currentTime;
            if (loopTime > ElapsedTime(lastTime))
            {
                Wait(loopTime - ElapsedTime(lastTime)); // seconds
            }
        }                       // end while

        DPRINTF(LOG_DEBUG, "end Autonomous");
        ShowActivity
            ("Autonomous end                                            ");

    }                           // end autonomous

        /**
	 * unchanged from SimpleDemo:
	 * Runs the motors under driver control with either tank or arcade steering selected
	 * by a jumper in DS Digin 0. 
	 */
    void OperatorControl(void)
    {
        char funcName[] = "OperatorControl";
        DPRINTF(LOG_DEBUG, "OperatorControl");
        GetWatchdog().Feed();

        while (1)
        {
            // determine if tank or arcade mode; default with no jumper is for tank drive
            if (ds->GetDigitalIn(ARCADE_MODE) == 0)
            {
                myRobot->TankDrive(leftStick, rightStick);      // drive with tank style
            }
            else
            {
                myRobot->ArcadeDrive(rightStick);       // drive with arcade style (use right stick)
            }
        }
    }                           // end operator control       
};

START_ROBOT_CLASS(TwoColorDemo);
