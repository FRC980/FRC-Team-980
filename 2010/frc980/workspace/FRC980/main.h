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
      //--- Class Variables ----------------------------------------------------
      
      //--- Instance Variables -------------------------------------------------
      
      //--- Constructors -------------------------------------------------------
      /*!\brief The Main contructor
       */
      Main();
      
      //--- Destructors --------------------------------------------------------
      /*!\brief The Main destructor
       */
      virtual ~Main();
      
      //--- Methods ------------------------------------------------------------
      
      /*!\brief The robot initialization
       */
      virtual void RobotInit();
      
      /*!\brief The disabled mode initialization method
       */
      virtual void DisabledInit();
      
      /*!\brief The disabled mode periodic method
       */
      virtual void DisabledPeriodic();
      
      /*!\brief The disabled mode continuous method
       */
      virtual void DisabledContinuous();
      
      /*!\brief The autonomous mode initialization method
       */
      virtual void AutonomousInit();
      
      /*!\brief The autonomous mode periodic method
       */
      virtual void AutonomousPeriodic();
      
      /*!\brief The autonomous mode continuous method
       */
      virtual void AutonomousContinuous();
      
      /*!\brief The teleoperated mode initialization method
       */
      virtual void TeleopInit();
      
      /*!\brief The teleoperated mode periodic method
       */
      virtual void TeleopPeriodic();
      
      /*!\brief The teleoperated mode continuous method
       */
      virtual void TeleopContinuous();
      
      //--- Friends ------------------------------------------------------------
};

#endif // MAIN_H
