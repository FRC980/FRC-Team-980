/******************************************************************************
*  Project          : FIRST Motor Controller
*  File Name        : VisionServoDemo.cpp
*  Contributors     : ELF
*  Creation Date    : Oct 4, 2008
*  Revision History : Source code & revision history maintained at
*                     sourceforge.WPI.edu
*  File Description : Demo program showing color tracking using servos
*/
/*---------------------------------------------------------------------------*/
/*        Copyright (c) FIRST 2008.  All Rights Reserved.
 *
 *  Open Source Software - may be modified and shared by FRC teams. The
 *  code must be accompanied by the FIRST BSD license file in
 *  $(WIND_BASE)/WPILib. */
/*---------------------------------------------------------------------------*/

#include <iostream.h>
#include "math.h"

#include "AxisCamera.h"
#include "BaeUtilities.h"
#include "FrcError.h"
#include "PCVideoServer.h"
#include "TrackAPI.h"
#include "WPILib.h"

// To locally enable debug printing: set VisionDemo_debugFlag to a 1, to
// disable set to 0
int  VisionServoDemo_debugFlag = 0;
#define DPRINTF if(VisionServoDemo_debugFlag)dprintf

#define PI 3.14159265358979

/**
 * This is a demo program showing the use of the color tracking API.
 * It uses the SimpleRobot class as a base of a robot application that
 * will automatically call your Autonomous and OperatorControl methods at
 * the right time as controlled by the switches on the driver station or
 * the field controls. Autonomous mode tracks color assuming camera is
 * mounted on a gimbal with two servos.
 */
class VisionServoDemo : public SimpleRobot
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

    PCVideoServer *m_pVideoServer;

    ParticleAnalysisReport par, par2;   // particle analysis report
    ColorReport cReport, cReport2;      // color report

    enum            // Driver Station jumpers to control program operation
    {
        ARCADE_MODE = 1, // Tank/Arcade jumper is on DS Input 1 (Jumper present is arcade)
        ENABLE_AUTONOMOUS = 2, // Autonomous/Teleop jumper is on DS Input 2 (Jumper present is autonomous)
    } jumpers;

  public:
    /**
     * Constructor for this robot subclass.
     * Create an instance of a RobotDrive with left and right motors
     * plugged into PWM ports 1 and 2 on the first digital module. The two
     * servos are PWM ports 3 and 4.
     */
    VisionServoDemo(void)
    {
        ds = DriverStation::GetInstance();
        myRobot = new RobotDrive(1, 2, 0.5); // robot will use PWM 1-2 for drive motors
        rightStick = new Joystick(1);   // create the joysticks
        leftStick = new Joystick(2);
        horizontalServo = new Servo(3); // create horizontal servo
        verticalServo = new Servo(4);   // create vertical servo
        servoDeadband = 0.01;   // move if > this amount
        framesPerSecond = 20;   // number of camera frames to get per second
        sinStart = 0.0;         // control whether to start panning up or down

        /* set up debug output:
         * DEBUG_OFF, DEBUG_MOSTLY_OFF, DEBUG_SCREEN_ONLY,
         * DEBUG_FILE_ONLY, DEBUG_SCREEN_AND_FILE
         */
        SetDebugFlag(DEBUG_FILE_ONLY);

        /* start the CameraTask  */
        if (StartCameraTask(framesPerSecond, 0, k320x240, ROT_180) == -1)
        {
            DPRINTF(LOG_ERROR,
                    "Failed to spawn camera task; exiting. Error code %s",
                    GetVisionErrorText(GetLastVisionError()));
        }

        m_pVideoServer = new PCVideoServer;

        /* allow writing to vxWorks target */
        Priv_SetWriteFileAllowed(1);

        /* stop the watchdog if debugging  */
        GetWatchdog().SetEnabled(false);
    }

    /**
     * Set servo positions (0.0 to 1.0) translated from normalized values
     * (-1.0 to 1.0).
     *
     * @param normalizedHorizontal Pan Position from -1.0 to 1.0.
     * @param normalizedVertical Tilt Position from -1.0 to 1.0.
     */
    void setServoPositions(float normalizedHorizontal,
                           float normalizedVertical)
    {
        float servoH = NormalizeToRange(normalizedHorizontal);

        /* narrow vertical range keep vertical servo from going too far */
        //float servoV = NormalizeToRange(normalizedVertical, 0.2, 0.8);
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
            // save new normalized vertical position
            verticalPosition = RangeToNormalized(servoV, 1);
        }
    }

    /**
     * Adjust servo positions (0.0 to 1.0) translated from normalized
     * values (-1.0 to 1.0).
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
        /* convert inputs to servo range */
        float servoH = NormalizeToRange(normDestH);

        /* compute vertical goal */
        float currentV = verticalServo->Get();
        float normCurrentV = RangeToNormalized(currentV, 1);
        float normDestV = normCurrentV + normDeltaVertical;
        if (normDestV > 1.0)
            normDestV = 1.0;
        if (normDestV < -1.0)
            normDestV = -1.0;

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
            // save new normalized vertical position
            verticalPosition = RangeToNormalized(servoV, 1);
        }
    }

    void Autonomous(void)
    {
        char funcName[] = "Autonomous";
        DPRINTF(LOG_DEBUG, "start VisionDemo autonomous");
        //GetWatchdog().Feed();

        // image data for tracking
        ColorMode mode = IMAQ_HSL;      // RGB or HSL
        //      TrackingThreshold td = GetTrackingData(RED, FLUORESCENT);
        TrackingThreshold td = GetTrackingData(GREEN, FLUORESCENT);

        int  panIncrement = 0;  // pan needs a 1-up number for each call

        DPRINTF(LOG_DEBUG, "SERVO - looking for COLOR %s ", td.name);

        /* initialize position and destination variables
         * position settings range from -1 to 1
         * setServoPositions is a wrapper that handles the conversion to
         * range for servo
         */
        horizontalDestination = 0.0; // final destination range -1.0 to +1.0
        verticalDestination = 0.0;

        // current position range -1.0 to +1.0
        horizontalPosition = RangeToNormalized(horizontalServo->Get(), 1);
        verticalPosition = RangeToNormalized(verticalServo->Get(), 1);

        // incremental tasking toward dest (-1.0 to 1.0)
        float incrementH, incrementV;

        // set servos to start at center position
        setServoPositions(horizontalDestination, verticalDestination);

        /* for controlling loop execution time */
        float loopTime = 0.05;
        double currentTime = GetTime();
        double lastTime = currentTime;

        double savedImageTimestamp = 0.0;

        bool foundColor = false;
        bool staleImage = false;

        while (IsAutonomous())
        {
            /* calculate gimbal position based on color found */
            if (FindColor
                (mode, &td.hue, &td.saturation, &td.luminance, &par,
                 &cReport))
            {
                foundColor = true;
                panIncrement = 0;       // reset pan
                if (par.imageTimestamp == savedImageTimestamp)
                {
                    // This image has been processed already,
                    // so don't do anything for this loop
                    staleImage = true;
                }
                else
                {
                    staleImage = false;
                    savedImageTimestamp = par.imageTimestamp;
                    // compute final H & V destination
                    horizontalDestination = par.center_mass_x_normalized;
                    verticalDestination = par.center_mass_y_normalized;
                }

//                ShowActivity("Found color   ");
            }
            else
            {                   // need to pan
                foundColor = false;
//                ShowActivity("No color found");
            }

            PrintReport(&cReport);

            if (foundColor && !staleImage)
            {
                /* Move the servo a bit each loop toward the destination.
                 * Alternative ways to task servos are to move immediately
                 * vs.  incrementally toward the final
                 * destination. Incremental method reduces the need for
                 * calibration of the servo movement while moving toward
                 * the target.
                 */
                incrementH = horizontalDestination - horizontalPosition;
                incrementV = verticalPosition - verticalDestination;
                adjustServoPositions(incrementH, incrementV);

                ShowActivity
                    ("** %s found: Servo: x: %f  y: %f  increment: %f  y: %f  ",
                     td.name, horizontalDestination, verticalDestination,
                     incrementH, incrementV);
            }
            else if (!staleImage)
            {
                /* pan to find color after a short wait to settle servos
                 * panning must start directly after panInit or timing
                 * will be off
                 */

                // adjust sine wave for panning based on last movement
                // direction
                if (horizontalDestination > 0.0)
                {
                    sinStart = PI / 2.0;
                }
                else
                {
                    sinStart = -PI / 2.0;
                }

                if (panIncrement == 3)
                {
                    panInit();
                }
                else if (panIncrement > 3)
                {
                    panForTarget(horizontalServo, sinStart);
                    /* Vertical action: center the vertical after several
                     * loops searching */
                    if (panIncrement == 20)
                    {
                        verticalServo->Set(0.5);
                    }
                }
                panIncrement++;
            } // end if found color

            // sleep to keep loop at constant rate
            // elapsed time can vary significantly due to debug printout
            currentTime = GetTime();
            lastTime = currentTime;
            if (loopTime > ElapsedTime(lastTime))
            {
                Wait(loopTime - ElapsedTime(lastTime));
            }

        } // end while

        myRobot->Drive(0.0, 0.0); // stop robot
        DPRINTF(LOG_DEBUG, "end autonomous");
        ShowActivity
            ("Autonomous end                                            ");

    } // end autonomous

    /**
     * unchanged from SimpleDemo:
     *
     * Runs the motors under driver control with either tank or arcade
     * steering selected by a jumper in DS Digin 0.
     *
     * added for vision:
     *
     * Adjusts the servo gimbal based on the color tracked.  Driving the
     * robot or operating an arm based on color input from gimbal-mounted
     * camera is currently left as an exercise for the teams.
     */
  void OperatorControl(void)
  {
        char funcName[] = "OperatorControl";
        DPRINTF(LOG_DEBUG, "OperatorControl");
        //GetWatchdog().Feed();

        while (IsOperatorControl())
        {
            setServoPositions(rightStick->GetX(), rightStick->GetY());
        }

        while (IsOperatorControl())
        {
            // determine if tank or arcade mode; default with no jumper is
            // for tank drive
            if (ds->GetDigitalIn(ARCADE_MODE) == 0)
            {
                // drive with tank style
                myRobot->TankDrive(leftStick, rightStick);
            }
            else
            {
                // drive with arcade style (use right stick)
                myRobot->ArcadeDrive(rightStick);
            }
        }
    } // end operator control
};

// entry point FRC_UserProgram_StartupLibraryInit
START_ROBOT_CLASS(VisionServoDemo);
