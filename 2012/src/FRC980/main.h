#ifndef MAIN_H
#define MAIN_H

#include <IterativeRobot.h>

//! The main code to run the robot
/*!\class Main
 *
 * The purpose of the Main class is to contain the different operational
 * modes of the robot.
 *
 */
class Main : public IterativeRobot
{
  private:

  public:
    //--- Instance Variables -----------------------------------------------

    //--- Constructors -----------------------------------------------------
    /*!\brief The Main contructor
     */
    Main(void);

    //--- Destructors ------------------------------------------------------
    /*!\brief The Main destructor
     */
    virtual ~ Main(void);

    //--- Methods ----------------------------------------------------------

    /*!\brief The robot initialization
     */
    virtual void RobotInit(void);

    /*!\brief The disabled mode initialization method
     */
    virtual void DisabledInit(void);

    /*!\brief The disabled mode periodic method
     */
    virtual void DisabledPeriodic(void);

    /*!\brief The disabled mode continuous method
     */
    virtual void DisabledContinuous(void);

    /*!\brief The autonomous mode initialization method
     */
    virtual void AutonomousInit(void);

    /*!\brief The autonomous mode periodic method
     */
    virtual void AutonomousPeriodic(void);

    /*!\brief The autonomous mode continuous method
     */
    virtual void AutonomousContinuous(void);

    /*!\brief The teleoperated mode initialization method
     */
    virtual void TeleopInit(void);

    /*!\brief The teleoperated mode periodic method
     */
    virtual void TeleopPeriodic(void);

    /*!\brief The teleoperated mode continuous method
     */
    virtual void TeleopContinuous(void);
};

#endif  // MAIN_H
