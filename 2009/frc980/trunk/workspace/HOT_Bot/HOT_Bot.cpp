#include <iostream.h>
#include <taskLib.h>
#include "string.h"
#include "WPILib.h"
#include "GamePad.h"
#include "HOT_Bot.h"
#include "HOT_PID.h"
// from vision demo
#include "math.h"
#include "AxisCamera.h"
#include "BaeUtilities.h"
#include "FrcError.h"
#include "TrackAPI.h"
#include "PCVideoServer.h"

#include "DashboardDataFormat.h"

/*
 * 2007 HOTBOT program using the IterativeRobot base class
 *
 *   Driver Station:
 *     Gamepad 1 - Driver Control
 *     Gamepad 2 - Arm Control
 *
 *   Robot:
 *              Digital Sidecar 1:
 *       PWM 1/3        - Left drive motor(s)
 *       PWM 2/4        - Right drive motor(s)
 *       PWM 5          - Hand top motor
 *       PWM 6          - Hand bottom motor
 *       PWM 7          - Arm motor
 *               PWM 9          - Camera pan servo
 *               PWM 10         - Camera tilt servo
 *
 *               Input 1/3      - Left encoder
 *               Input 2/4      - Right encoder
 *               Input 5        - Compressor pressure switch
 *
 *               Relay 1        - Compressor relay
 *
 *              Analog Input Module 1:
 *               Input 1        - Gyro
 *               Input 2        - Arm potentiometer
 *
 *              Solenoid Module 1:
 *               Output 1       - Pneumatics solenoid
 *
 *
 */

class HOT_Bot : public IterativeRobot
{
  private:

    // Declare pointer to Driver Station object
    DriverStation * m_ds;       // driver station
    UINT32 m_priorPacketNumber; // most recent packet number from the DS
    UINT8 m_dsPacketsReceivedInCurrentSecond;   // number of DS packets received in the current second

    // Declare pointer to Dashboard object
    Dashboard *m_dashboard;
    // Define Dashboard Structure
    DashboardDataFormat dashboardDataFormat;

    // Declare pointers to GamePad objects
    GamePad *m_gamePad1;        // gamepad 1 (drive control)
    GamePad *m_gamePad2;        // gamepad 2 (arm control)

    // Declare pointers to Drive Motor objects
    Jaguar *m_driveLeft;        // left drive motor
    Jaguar *m_driveRight;       // right drive motor

    // Declare pointer to Robot Drive object
    RobotDrive *m_robotDrive;   // robot drive

    // Declare pointers to Hand Motor objects
    Victor *m_handTopMotor;     // hand top motor
    Victor *m_handBottomMotor;  // hand bottom motor

    // Declare pointers to Arm Motor objects
    Jaguar *m_armMotor;         // arm motor

    // Declare pointers Camera Servo objects
    Servo *m_cameraPan;         // camera pan servo
    Servo *m_cameraTilt;        // camera tilt servo

    // Declare pointers to Compressor and Solenoid objects
    Compressor *m_compressor;   // air compressor
    Solenoid *m_solenoid;       // pneumatics solenoid

    // Declare pointers to Encoder objects
    Encoder *m_leftEncoder;     // left-side encoder
    Encoder *m_rightEncoder;    // right-side encoder

    Encoder *m_E3;
    Encoder *m_E4;
    Encoder *m_E5;
    Encoder *m_E6;

    // Declare pointers to Gyro and Arm Potentiometer objects
    Gyro *m_gyro;               // gyro
    AnalogChannel *m_armPot;    // arm potentiometer

    // Declare pointers to PID control objects
    HOT_PID *m_armPID;          // arm PID control
    HOT_PID *m_steerPID;        // steer PID control
    HOT_PID *m_drivePID;        // drive PID control

    // Define Axis 206 Camera Structures
    TrackingThreshold m_tdata;  // image data for tracking
    ParticleAnalysisReport m_pReport;   // particle analysis report
    ColorReport m_cReport;      // color report

    // Count the number of periodic loops done
    unsigned int m_autoPeriodicLoops;
    unsigned int m_disabledPeriodicLoops;
    unsigned int m_telePeriodicLoops;

    // Define autonomous mode control variables
    unsigned int m_autoTime;    // autonomous mode clock (ms)
    unsigned int m_autoState;   // autonomous mode state
    unsigned int m_autoProgramNumber;   // autonomous mode program number

    // Define encoder information variables
    int  m_leftEncoderCount;    // left encoder position (counts)
    int  m_rightEncoderCount;   // right encoder position (counts)
    float m_leftEncoderVelocity;        // left encoder velocity (counts/second)
    float m_rightEncoderVelocity;       // right encoder velocity (counts/second)

    // Define orientation variables
    float m_bearing;            // Direction we want to go (degrees)
    float m_heading;            // Direction we're headed (degrees)

    // last GampPad front button positions
    bool p1_B5old;
    bool p1_B6old;
    bool p1_B7old;
    bool p1_B8old;
    bool p2_B5old;
    bool p2_B6old;
    bool p2_B7old;
    bool p2_B8old;

    PCVideoServer *pc;

  public:
/**
 * Constructor for this "HOT_Bot" Class.
 */
    HOT_Bot(void)
    {
        printf("\nEntering HotBot Constructor\n");

        // Define gamepad objects at USB 1 and 2 on the Drivers Station
        m_gamePad1 = new GamePad(1);
        m_gamePad2 = new GamePad(2);

        // Acquire the Driver Station object
        m_ds = DriverStation::GetInstance();

        // Define dashboard object
        m_dashboard = &m_ds->GetDashboardPacker();

        // Define drive motor objects by Jaguars on PWM 1 and 2
        m_driveLeft = new Jaguar(1);
        m_driveRight = new Jaguar(2);

        // Define robot drive object with two-motor drive
        m_robotDrive = new RobotDrive(m_driveLeft, m_driveRight);

        // Define hand motor objects by Jaguars on PWM 5 and 6
        m_handTopMotor = new Victor(5);
        m_handBottomMotor = new Victor(6);

        // Define armMotor object by Jaguar on PWM 7
        m_armMotor = new Jaguar(7);

        // Define camera servo objects on PWM 9 and 10
        m_cameraPan = new Servo(9);
        m_cameraTilt = new Servo(10);

        // Define compressor object with
        //              pressure switch on digital input 5
        //              compressor relay on relay output 1
        m_compressor = new Compressor(5, 1);

        // Define pneumatics solenoid object on solenoid output 1
        m_solenoid = new Solenoid(1);

        // Define encoder objects on digital inputs to match numbering of
        // drive motor PWMs
        m_leftEncoder = new Encoder(1, 3, false);       // left-side encoder
        m_rightEncoder = new Encoder(2, 4, true);       // right-side encoder
        m_E3 = new Encoder(5, 6, true);
        m_E4 = new Encoder(7, 8, true);
        m_E5 = new Encoder(9, 10, true);
        m_E6 = new Encoder(11, 12, true);

        // Define gyro object on analog input 1
        m_gyro = new Gyro(1);
        m_gyro->SetSensitivity(GYRO_SENSITIVITY);

        // Define arm pot object on analog input 2
        m_armPot = new AnalogChannel(2);

        // Define arm PID control object
        m_armPID = new HOT_PID(2.0, 0.0, 0.0, 0.01); // Kp, Ki, Kd, tolerance

        // Define steering PID control object
        m_steerPID = new HOT_PID(0.0, 0.0, 0.0, 0.02); // Kp, Ki, Kd, tolerance

        // Define driving PID control object
        m_drivePID = new HOT_PID(0.0, 0.0, 0.0, 0.05); // Kp, Ki, Kd, tolerance

        // Set debug type for vision-camera code
        SetDebugFlag(DEBUG_SCREEN_ONLY); // dprintf output goes to terminal/console

        // Start the camera task
        if (StartCameraTask(10, 0, k160x120, ROT_0) == -1)
        {
            dprintf(LOG_ERROR, "Failed to spawn camera task; Error code %s",
                    GetVisionErrorText(GetLastVisionError()));
        }

        // 2006-2007 FRC Green Lamp threshold values
        strcpy(m_tdata.name, "GREEN LAMP");
        m_tdata.saturation.minValue = 200;
        m_tdata.saturation.maxValue = 255;
        m_tdata.luminance.minValue = 100;
        m_tdata.luminance.maxValue = 160;
        m_tdata.hue.minValue = 70;
        m_tdata.hue.maxValue = 100;

        // Start the task serving images to PC
        pc = new PCVideoServer();

        GetWatchdog().SetExpiration(200);

        printf("Leaving HotBot Constructor\n\n");

    }

    /*************************** Init Routines ******************************/

    void RobotInit(void)
    {
        // Actions which would be performed once (and only once) upon
        // initialization of the robot would be put here.
        printf("\nEntering RobotInit\n");

        // set all GamePad buttons to not-pressed
        p1_B5old = false;
        p1_B6old = false;
        p1_B7old = false;
        p1_B8old = false;
        p2_B5old = false;
        p2_B6old = false;
        p2_B7old = false;
        p2_B8old = false;

        m_compressor->Start();

        // set autonomous mode program number
        m_autoProgramNumber = 0;

        printf("Leaving RobotInit\n");
    }

    void DisabledInit(void)
    {
        printf("\nEntering DisabledInit\n");

        m_disabledPeriodicLoops = 0;    // Reset the disabled mode loop counter

        // init Autonomous Program if a jumper is in place
        if (m_ds->GetDigitalIn(1))
            m_autoProgramNumber = 1;
        if (m_ds->GetDigitalIn(2))
            m_autoProgramNumber = 2;
        if (m_ds->GetDigitalIn(3))
            m_autoProgramNumber = 3;
        if (m_ds->GetDigitalIn(4))
            m_autoProgramNumber = 4;
        if (m_ds->GetDigitalIn(5))
            m_autoProgramNumber = 5;
        if (m_ds->GetDigitalIn(6))
            m_autoProgramNumber = 6;
        if (m_ds->GetDigitalIn(7))
            m_autoProgramNumber = 7;
        if (m_ds->GetDigitalIn(8))
            m_autoProgramNumber = 8;

        printf("Leaving DisabledInit\n");
    }

    void AutonomousInit(void)
    {
        printf("\nEntering AutonomousInit\n");

        // Reset autonomous control information
        m_autoPeriodicLoops = 0;        // loop counter
        m_autoTime = 0;         // time
        m_autoState = 0;        // state

        // Initialize arm PID control
        m_armPID->Reset();
        m_armPID->Enable();
        m_armPID->SetSPLimits(5.0, 0.0); // Set Point (volts)
        m_armPID->SetPVLimits(5.0, 0.0); // Process Variable, arm pot input (volts)
        m_armPID->SetMVLimits(+1.0, -1.0); // Manipulated Variable, Jaguar drives arm motor

        // Reset gyro
        m_gyro->Reset();

        // Start and Reset encoders
        m_leftEncoder->Start();
        m_rightEncoder->Start();
        m_leftEncoder->Reset();
        m_rightEncoder->Reset();

        // Initialize encoder variables
        m_leftEncoderCount = 0;
        m_rightEncoderCount = 0;
        m_leftEncoderVelocity = 0.0;
        m_rightEncoderVelocity = 0.0;

        // Initialize orientation variables
        m_bearing = 0.0;
        m_heading = 0.0;

        // Center the camera
        m_cameraTilt->Set(0.5);
        m_cameraPan->Set(0.5);

        printf("Autonomous Program Number = %u\n", m_autoProgramNumber);
        printf("Leaving DisabledInit\n");
    }

    void TeleopInit(void)
    {
        printf("\nEntering TeleopInit\n");

        m_telePeriodicLoops = 0; // Reset the loop counter for teleop mode
        m_dsPacketsReceivedInCurrentSecond = 0; // Reset the number of dsPackets in current second
        m_priorPacketNumber = 0;

        // Initialize arm PID control
        m_armPID->Reset();
        m_armPID->Enable();
        m_armPID->SetSPLimits(5.0, 0.0);        // Set Point (volts)
        m_armPID->SetPVLimits(5.0, 0.0);        // Process Variable, arm pot input (volts)
        m_armPID->SetMVLimits(+1.0, -1.0);      // Manipulated Variable, Jaguar drives arm motor

        // Simbotics says, do this or you'll be sorry!
        m_robotDrive->TankDrive(0.0, 0.0);
        m_handTopMotor->Set(0.0);
        m_handBottomMotor->Set(0.0);
        m_armMotor->Set(0.0);

        printf("Leaving TeleopInit\n\n");
    }

    /************************** Periodic Routines ***************************/

    void DisabledPeriodic(void)
    {
        // feed the user watchdog at every period when disabled
        GetWatchdog().Feed();

        // increment the number of disabled periodic loops completed
        m_disabledPeriodicLoops++;

        // while disabled, printout the number of seconds of disabled so far
        /*              if ((m_disabledPeriodicLoops % (UINT32)GetLoopsPerSec()) == 0) {
         * printf("Disabled seconds:%5.0f  program:%4u\r\n", (m_disabledPeriodicLoops / GetLoopsPerSec()), m_autoProgramNumber);
         *
         * m_dashboard->Printf("It's been %f seconds, according to the FPGA.\n", GetClock());
         * UpdateDashboard();
         *
         * }
         */
        // print to the dashboard
        m_dashboard->Printf("%5.0f seconds, Autonomous Program:%4u\n",
                            GetClock(), m_autoProgramNumber);
        UpdateDashboard();

        /* ------------------------------------------------------------------
         * GamePad1 Autonomous mode program number selection
         * ------------------------------------------------------------------
         * Holding B5 and toggling B6 increments by 1
         * Holding B5 and toggling B8 decrements by 1
         * Holding B7 and toggling B6 increments by 10
         * Holding B7 and toggling B8 decrements by 10
         */
        if (m_gamePad1->GetButton05() && m_gamePad1->GetButton06()
            && (!p1_B6old))
        {
            m_autoProgramNumber++;
        }
        if (m_gamePad1->GetButton05() && m_gamePad1->GetButton08()
            && (!p1_B8old))
        {
            m_autoProgramNumber--;
        }
        if (m_gamePad1->GetButton07() && m_gamePad1->GetButton06()
            && (!p1_B6old))
        {
            m_autoProgramNumber += 10;
        }
        if (m_gamePad1->GetButton07() && m_gamePad1->GetButton08()
            && (!p1_B8old))
        {
            m_autoProgramNumber -= 10;
        }

        // Save button states for next time through the loop
        p1_B5old = m_gamePad1->GetButton05();
        p1_B6old = m_gamePad1->GetButton06();
        p1_B7old = m_gamePad1->GetButton07();
        p1_B8old = m_gamePad1->GetButton08();
        p2_B5old = m_gamePad2->GetButton05();
        p2_B6old = m_gamePad2->GetButton06();
        p2_B7old = m_gamePad2->GetButton07();
        p2_B8old = m_gamePad2->GetButton08();

    }

    void AutonomousPeriodic(void)
    {
        // feed the user watchdog at every period when in autonomous
        GetWatchdog().Feed();

        m_autoPeriodicLoops++;

        switch (m_autoProgramNumber)
        {
        case 0:
            AutonomousProgram_00();     // Sensor Display
            break;

        case 1:
            AutonomousProgram_01();     // Gyro Demo
            break;

        case 2:
            AutonomousProgram_02();     // Encoder Demo
            break;

        case 3:
            AutonomousProgram_03();     // Gyro+Encoder Demo (zig-zag)
            break;

        case 4:
            AutonomousProgram_04();     // 2007 Rack n' Roll
            break;
        }

        // Update autonomous loop time variable
        // Autonomous loop runs at 200Hz or every 5 ms
        m_autoTime += 5;
    }

    /*
     * print sensor values, don't move:  gyro, left and right encoders, camera
     */
    void AutonomousProgram_00(void)
    {
        static double savedImageTimestamp;

        m_dashboard->Printf("%5.0f seconds\n", GetClock());

        // get gyro and encoder data
        m_heading = m_gyro->GetAngle();
        m_leftEncoderCount = m_leftEncoder->Get();
        m_rightEncoderCount = m_rightEncoder->Get();

        // print gyro and encoder data
        if ((m_autoPeriodicLoops % 40) == 0)    // print twice per second
        {
            printf("%5.0f seconds, ", GetClock());
            printf("gyro heading:%6.2f degrees, encoder:%6d%6d\n",
                   m_heading, m_leftEncoderCount, m_rightEncoderCount);
        }
        m_dashboard->Printf("gyro heading:%6.2f degrees, encoder:%6d%6d\n",
                            m_heading, m_leftEncoderCount,
                            m_rightEncoderCount);

        // get and print camera data
        if (FindColor
            (IMAQ_HSL, &m_tdata.hue, &m_tdata.saturation,
             &m_tdata.luminance, &m_pReport, &m_cReport)
            && m_pReport.particleToImagePercent <
            MAX_PARTICLE_TO_IMAGE_PERCENT
            && m_pReport.particleToImagePercent >
            MIN_PARTICLE_TO_IMAGE_PERCENT)
        {
            if (m_pReport.imageTimestamp != savedImageTimestamp)
            {
                savedImageTimestamp = m_pReport.imageTimestamp;

                if ((m_autoPeriodicLoops % 40) == 0) // print twice per second
                {
                    printf
                        ("%s     found. x:%6.3f, y:%6.3f, area:%7.0f, percent:%8.3f ",
                         m_tdata.name, m_pReport.center_mass_x_normalized,
                         m_pReport.center_mass_y_normalized,
                         m_pReport.particleArea,
                         m_pReport.particleToImagePercent);
                }
                m_dashboard->
                    Printf
                    ("%s     found. x:%6.3f, y:%6.3f, area:%7.0f, percent:%8.3f ",
                     m_tdata.name, m_pReport.center_mass_x_normalized,
                     m_pReport.center_mass_y_normalized,
                     m_pReport.particleArea,
                     m_pReport.particleToImagePercent);
            }
        }
        else
        {
            if ((m_autoPeriodicLoops % 5) == 0) // print twice per second
            {
                printf("%s NOT found.", m_tdata.name);
            }
            m_dashboard->Printf("%s NOT found.", m_tdata.name);

        }

        UpdateDashboard();
    }

    /*
     * Gyro Demonstration Program: drive in a straight line
     */
    void AutonomousProgram_01(void)
    {
        float drive = 0.5;      // ArcadeDrive drive
        float steer = 0.0;      // ArcadeDrive steer

        switch (m_autoState)
        {
        case 0:                // Initialize
            // Initialize steering PID
            m_steerPID->Reset();
            m_steerPID->Enable();
            m_steerPID->SetSPLimits(+90.0, -90.0);      // Set Point, bearing in degrees
            m_steerPID->SetPVLimits(+90.0, -90.0);      // Process Variable, heading in degrees
            m_steerPID->SetMVLimits(+1.0, -1.0);        // Manipulated Variable, ArcadeDrive steer
            m_steerPID->SetGains(0.01, 0.0, 0.0);
            m_bearing = 0.0;    // Orientation we want to go (degrees)
            m_autoState = 1;
            break;

        case 1:                // drive
            m_heading = -m_gyro->GetAngle();    // gyro measures positive counterclockwise
            steer = m_steerPID->GetMV(m_bearing, m_heading);
            break;

        default:
            break;

        }

        // Drive the robot
        m_robotDrive->ArcadeDrive(drive, steer, false);

        // print to the console at 5 Hz (I like the cRIO serial port)
        if ((m_autoPeriodicLoops % 40) == 0)
        {
            printf("%5.0f seconds, State: %u,  ", GetClock(), m_autoState);
            printf(" heading:%6.2f, steer:%6.3f\r\n", m_heading, steer);
        }
        // print to the dashboard
        m_dashboard->Printf("%5.0f seconds, State: %u\n", GetClock(),
                            m_autoState);
        m_dashboard->Printf(" heading:%6.2f degrees\r\n", m_heading);
        UpdateDashboard();
    }

    /*
     * Encoder Demonstration Program: Drive a specified distance (and
     * maybe travel somewhat in a straight line)
     */
    void AutonomousProgram_02(void)
    {
        float drive = 0;        // ArcadeDrive drive
        float steer = 0;        // ArcadeDrive steer
        float distance;         // how far have we gone ? (mm)

        switch (m_autoState)
        {
        case 0:                // drive to the target and stop
            drive = 0.5;
            steer = 0.0;
            // how far have we gone?
            m_leftEncoderCount = m_leftEncoder->Get();
            m_rightEncoderCount = m_rightEncoder->Get();
            distance =
                MM_PER_COUNT * (m_leftEncoderCount +
                                m_rightEncoderCount) / 2.0;
            // move on to next state after reaching target
            if (distance > 6000.)
            {
                // stop the robot
                drive = 0.0;
                steer = 0.0;
                m_autoState = 1;
            }
            break;

        default:
            break;
        }

        // Drive the robot
        m_robotDrive->ArcadeDrive(drive, steer, false);

        // print to the console at 5 Hz (I like the cRIO serial port)
        if ((m_autoPeriodicLoops % 40) == 0)
        {
            printf("%5.0f seconds, State: %u,  ", GetClock(), m_autoState);
            printf("encoder:%6d%6d, distance:%6.0f\n", m_leftEncoderCount,
                   m_leftEncoderCount, distance);
        }
        // print to the dashboard
        m_dashboard->Printf("%5.0f seconds, State: %u\n", GetClock(),
                            m_autoState);
        m_dashboard->Printf("encoder:%6d%6d, distance:%6.0f\n",
                            m_leftEncoderCount, m_leftEncoderCount,
                            distance);
        UpdateDashboard();
    }

    /*
     *  Gyro/Encoder Demonstration Program: Drive in a zig-zag pattern
     */
    void AutonomousProgram_03(void)
    {
        float drive = 0;        // ArcadeDrive drive
        float steer = 0;        // ArcadeDrive steer
        float distance;         // how far have we gone ? (mm)

        switch (m_autoState)
        {
        case 0:                // Initialize
            // Initialize steering PID
            m_steerPID->Reset();
            m_steerPID->Enable();
            m_steerPID->SetSPLimits(+90.0, -90.0);      // Set Point, bearing in degrees
            m_steerPID->SetPVLimits(+90.0, -90.0);      // Process Variable, heading in degrees
            m_steerPID->SetMVLimits(+1.0, -1.0);        // Manipulated Variable, ArcadeDrive steer
            m_steerPID->SetGains(0.01, 0.0, 0.0);
            m_bearing = 30.0;   // degrees
            m_autoState = 1;
            break;

        case 1:                // try to drive straight
            m_heading = -m_gyro->GetAngle();    // degrees
            drive = 0.5;
            steer = m_steerPID->GetMV(m_bearing, m_heading);

            // how far have we gone?
            distance =
                MM_PER_COUNT * (m_leftEncoder->Get() +
                                m_rightEncoder->Get()) / 2.0;
            // move on to next state after reaching target
            if (distance > 2000.)
            {
                // reset encoders
                m_leftEncoder->Reset();
                m_rightEncoder->Reset();
                // set the bearing
                m_bearing = -30.0;      // degrees
                m_autoState = 2;
            }
            break;

        case 2:                // try to drive straight
            m_heading = -m_gyro->GetAngle();    // degrees
            drive = 0.5;
            steer = m_steerPID->GetMV(m_bearing, m_heading);

            // how far have we gone?
            distance =
                MM_PER_COUNT * (m_leftEncoder->Get() +
                                m_rightEncoder->Get()) / 2.0;
            // move on to next state after reaching target
            if (distance > 4000.)
            {
                // reset encoders
                m_leftEncoder->Reset();
                m_rightEncoder->Reset();
                // set the bearing
                m_bearing = 30.0;       // degrees
                m_autoState = 3;
            }
            break;

        case 3:                // try to drive straight
            m_heading = -m_gyro->GetAngle();    // degrees
            drive = 0.5;
            steer = m_steerPID->GetMV(m_bearing, m_heading);

            // how far have we gone?
            distance =
                MM_PER_COUNT * (m_leftEncoder->Get() +
                                m_rightEncoder->Get()) / 2.0;
            // move on to next state after reaching target
            if (distance > 2000.)
            {
                // stop all engines!
                drive = 0.0;
                steer = 0.0;
                m_autoState = 4;
            }
            break;

        default:
            break;
        }

        // Drive the robot
        m_robotDrive->ArcadeDrive(drive, steer, false);

        // print to the console at 5 Hz (I like the cRIO serial port)
        if ((m_autoPeriodicLoops % 40) == 0)
        {
            printf("%5.0f seconds, State: %u,  ", GetClock(), m_autoState);
            printf
                ("distance:%6.0f, heading:%6.2f degrees, bearing:%6.2f degrees\n",
                 distance, m_heading, m_bearing);
        }
        // print to the dashboard
        m_dashboard->Printf("%5.0f seconds, State: %u\n", GetClock(),
                            m_autoState);
        m_dashboard->
            Printf
            ("distance:%6.0f, heading:%6.2f degrees, bearing:%6.2f degrees\n",
             distance, m_heading, m_bearing);
        UpdateDashboard();
    }

    /*
     * 2007 Rack n' Roll Autonomous Demonstration Program
     * Raise the arm, find the green lamp, drive to the green lamp, lower
     * the arm
     */
    void AutonomousProgram_04(void)
    {
        static double savedImageTimestamp;
        float drive = 0;        // ArcadeDrive drive
        float steer = 0;        // ArcadeDrive steer

        m_dashboard->Printf("%5.0f seconds, State: %u\n", GetClock(),
                            m_autoState);

        switch (m_autoState)
        {
        case 0:                // Initialize
            // Initialize arm PID
            m_armPID->Reset();
            // Initialize steering PID
            m_steerPID->Reset();
            m_steerPID->Enable();
            m_steerPID->SetSPLimits(+1.0, -1.0);        // Set Point, image coordinates
            m_steerPID->SetPVLimits(+1.0, -1.0);        // Process Variable, image coordinates
            m_steerPID->SetMVLimits(+0.3, -0.3);        // Manipulated Variable, ArcadeDrive steering
            m_steerPID->SetGains(1.0, 0.1, 5.0);
            // Initialize drive PID
            m_drivePID->Reset();
            m_drivePID->Enable();
            m_drivePID->SetSPLimits(10.0, 0.0); // Set Point, image particleToImagePercent
            m_drivePID->SetPVLimits(10.0, 0.0); // Process Variable, image particleToImagePercent
            m_drivePID->SetMVLimits(+0.5, -0.5);        // Manipulated Variable, ArcadeDrive drive
            m_drivePID->SetGains(1.0, 0.0, 1.0);
            m_autoState = 1;
            break;

        case 1:                // Raise the arm
            m_armMotor->Set(m_armPID->
                            GetMV(ARM_POSITION_BOTTOM,
                                  m_armPot->GetVoltage()));
            if (m_armPID->OnTarget())
            {
                m_armMotor->Set(0.0);   // turn off arm motor
                m_autoState = 2;
            }
            break;

        case 2:   // Look for the green lamp, and drive towards it if seen
            if (FindColor
                (IMAQ_HSL, &m_tdata.hue, &m_tdata.saturation,
                 &m_tdata.luminance, &m_pReport, &m_cReport)
                && m_pReport.particleToImagePercent <
                MAX_PARTICLE_TO_IMAGE_PERCENT
                && m_pReport.particleToImagePercent >
                MIN_PARTICLE_TO_IMAGE_PERCENT)
            {
                if (m_pReport.imageTimestamp != savedImageTimestamp)
                {
                    savedImageTimestamp = m_pReport.imageTimestamp;

                    m_dashboard->
                        Printf
                        ("%s     found. x:%6.3f, y:%6.3f, area:%7.0f, percent:%8.3f\n",
                         m_tdata.name, m_pReport.center_mass_x_normalized,
                         m_pReport.center_mass_y_normalized,
                         m_pReport.particleArea,
                         m_pReport.particleToImagePercent);

                    // get ArcadeDrive drive and steer inputs
                    steer =
                        m_steerPID->GetMV(0.0,
                                          m_pReport.
                                          center_mass_x_normalized);
                    drive =
                        m_drivePID->GetMV(1.0,
                                          m_pReport.particleToImagePercent);

                    // check to see if we've 'arrived'
                    if (m_drivePID->OnTarget() && m_steerPID->OnTarget())
                    {
                        // stop the robot
                        drive = 0.0;
                        steer = 0.0;
                        m_autoState = 3;
                    }
                }
            }
            else
            {
                m_dashboard->Printf("%s NOT found.\n", m_tdata.name);
                // spin about slowly if the camera (and robot) does not
                // see the green lamp
                drive = 0.0;
                steer = 0.3;
            }
            break;

        case 3:          // Lower the arm back down to the ground position
            m_armMotor->Set(m_armPID->
                            GetMV(ARM_POSITION_GROUND,
                                  m_armPot->GetVoltage()));
            if (m_armPID->OnTarget())
            {
                m_armMotor->Set(0.0);   // turn off arm motor
                m_autoState = 4;
            }
            break;

        default:
            break;
        }

        // Drive the robot
        m_robotDrive->ArcadeDrive(drive, steer, false);

        // Print to the dashboard
        m_dashboard->Printf("drive:%6.3f, steer:%6.3f\n", drive, steer);
        UpdateDashboard();
    }

    void TeleopPeriodic(void)
    {
        // feed the user watchdog at every period when in autonomous
        GetWatchdog().Feed();

        // increment the number of teleop periodic loops completed
        m_telePeriodicLoops++;

        // tank drive using GamePad 1 left and right joystick y-axes
        m_robotDrive->TankDrive(m_gamePad1->GetLeftY(),
                                m_gamePad1->GetRightY());

        // PID control of arm
        if (m_gamePad2->GetButton01())
        {
            m_armMotor->Set(m_armPID->
                            GetMV(ARM_POSITION_GROUND,
                                  m_armPot->GetVoltage()));
        }
        else if (m_gamePad2->GetButton02())
        {
            m_armMotor->Set(m_armPID->
                            GetMV(ARM_POSITION_BOTTOM,
                                  m_armPot->GetVoltage()));
        }
        else if (m_gamePad2->GetButton03())
        {
            m_armMotor->Set(m_armPID->
                            GetMV(ARM_POSITION_MIDDLE,
                                  m_armPot->GetVoltage()));
        }
        else if (m_gamePad2->GetButton04())
        {
            m_armMotor->Set(m_armPID->
                            GetMV(ARM_POSITION_TOP,
                                  m_armPot->GetVoltage()));
        }
        else if (m_gamePad2->GetButton07() && m_gamePad2->GetButton08())
        {
            m_armMotor->Set(m_armPID->
                            GetMV(ARM_POSITION_STOWED,
                                  m_armPot->GetVoltage()));
        }
        // manual control of arm: left joystick
        else
        {
            m_armMotor->Set(m_gamePad2->GetLeftY());
        }

        /*
         * Run hand motor and solenoid blocks at 50Hz, since DS data are
         * received at that rate.
         */
        if ((m_telePeriodicLoops % 4) == 0)
        {
            // hand motor control: right joystick
            if (m_gamePad2->GetButton05())
            {                   // hold button 5 to grab or release tube
                m_handTopMotor->Set(-m_gamePad2->GetRightY());
                m_handBottomMotor->Set(m_gamePad2->GetRightY());
            }
            else                // release button 5 to tilt tube up and down
            {
                m_handTopMotor->Set(-m_gamePad2->GetRightY());
                m_handBottomMotor->Set(-m_gamePad2->GetRightY());
            }

            if (m_gamePad1->GetButton09())
            {
                m_solenoid->Set(1);
            }
            else
            {
                m_solenoid->Set(0);
            }
        }

        // print to the console (I like the cRIO serial port)
        if ((m_telePeriodicLoops % 40) == 0)    // twice per second
        {
            printf("leftY = %5.2f, rightY = %5.2f,  ",
                   m_gamePad1->GetLeftY(), m_gamePad1->GetRightY());
            printf("arm_PV = %5.2f, arm_MV = %5.2f\n",
                   m_armPot->GetVoltage(), m_armMotor->Get());
        }

        m_dashboard->Printf("%5.0f seconds, armPot %5.2f\n", GetClock(),
                            m_armPot->GetVoltage());
        UpdateDashboard();
    }

    /**
     * Send data to the dashboard
     */
    void UpdateDashboard(void)
    {
        dashboardDataFormat.m_AnalogChannels[0][1] = m_armPot->GetVoltage();
        dashboardDataFormat.m_PWMChannels[0][0] = m_driveLeft->GetRaw();
        dashboardDataFormat.m_PWMChannels[0][1] = m_driveRight->GetRaw();
        dashboardDataFormat.m_PWMChannels[0][2] = 0;
        dashboardDataFormat.m_PWMChannels[0][3] = 0;
        dashboardDataFormat.m_PWMChannels[0][4] = m_handTopMotor->GetRaw();
        dashboardDataFormat.m_PWMChannels[0][5] =
            m_handBottomMotor->GetRaw();
        dashboardDataFormat.m_PWMChannels[0][6] = m_armMotor->GetRaw();
        dashboardDataFormat.m_PWMChannels[0][7] = 0;
        dashboardDataFormat.m_PWMChannels[0][8] = m_cameraPan->GetRaw();
        dashboardDataFormat.m_PWMChannels[0][9] = m_cameraTilt->GetRaw();
        dashboardDataFormat.PackAndSend();
    }

};

START_ROBOT_CLASS(HOT_Bot);
