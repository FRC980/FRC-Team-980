#ifndef SMARTDRIVE_H
#define SMARTDRIVE_H

#define SD_ID_LEFT  0
#define SD_ID_RIGHT 1

typedef unsigned char uint8_t;

#include <SpeedController.h>

class Encoder;
class Timer;

//! The smart drive controls for the robot
/*!\class SmartDrive
 *
 * The purpose of the SmartDrive class is to provide intelligent driving
 * capabilities to the driving of the robot
 *    
 */
class SmartDrive : public SpeedController
{
   private:
      //--- Class Variables ----------------------------------------------------
      SpeedController* m_psc;
      Encoder* m_pEncDrive;
      Encoder* m_pEncFollow;
      Timer* m_pTimer;
      Notifier *m_controlLoop;
      
      //--- Instance Variables -------------------------------------------------
      const uint8_t m_ku8ID;
      const double m_kVelP, m_kVelI;
      const double m_kCorP, m_kCorI;
      const double m_kAclP, m_kAclI;
      bool m_bUseAcl;
      bool m_bUseSlip;
      const float  m_kfPeriod;
      
      double m_dCmdSpeed;
      bool   m_bEnabled;
      
      double m_dPrevMotorVel;     /*!<\brief previous motor velocity */
      double m_dPrevRobotVel;     /*!<\brief previous follow-wheel velocity */
      double m_dVelInt;           /*!<\brief integral term in velocity PID loop */
      double m_dCorInt;           /*!<\brief integral term in correction PID loop */
      double m_dAclInt;           /*!<\brief integral term in acceleration PID loop */
      
      double m_dPrevMotorCount;
      double m_dPrevRobotCount;
      
      //--- Methods ------------------------------------------------------------
      
      /*!\brief Set the command speed
       * Call the Calculate method as a non-static method. This avoids having to
       * prepend all local variables in that method with the class pointer. This
       * way the "this" pointer will be set up and class variables can be called
       * more easily.  This method is static and called by the Notifier class.
       * 
       * \param controller the address of the PID controller object to use in
       * the background loop
       * 
       */
      static void CallCalculate(void *controller);
      
      /*!\brief Implement the PID loops
       *  This code implements 3 linked PID loops (technically PI, as there
       *  is no D term).  The VELOCITY loop tries to make the motor run at
       *  the commanded speed.  The ACCELERATION loop tries to maximize
       *  acceleration.  The SLIPPAGE loop (aka "correction" loop) tries to
       *  minimize slippage, (as measured by the difference between the motor
       *  speed and follow wheel speed).
       */
      void Calculate(void);
      
      DISALLOW_COPY_AND_ASSIGN(SmartDrive);
      
   public:
      //--- Instance Variables -------------------------------------------------
      
      //--- Constructors -------------------------------------------------------
      /*!\brief The SmartDrive contructor
       * \param id The ID number of the SmartDrive
       * \param kvp The velocity PID constant
       * \param kvi The velocity PID constant
       * \param kcp The correction PID constant
       * \param kci The correction PID constant
       * \param kap The acceleration PID constant
       * \param kai The acceleration PID constant
       * \param kbacl The acceleration PID constant
       * \param psc The Speed Controller
       * \param pEncDrive The Drive Encoder
       * \param pEncFollow The Follow Encoder
       * \param period The control loop period
       */
      SmartDrive(uint8_t id,
                 double kvp, double kvi, // velocity PID constants
                 double kcp, double kci, // correction PID constants
                 double kap, double kai, bool kbacl, // accel PID constants
                 SpeedController* psc,
                 Encoder* pEncDrive,
                 Encoder* pEncFollow,
                 float period = 0.010);
      
      //--- Destructors --------------------------------------------------------
      /*!\brief The SmartDrive destructor
       */
      virtual ~SmartDrive();
      
      //--- Methods ------------------------------------------------------------
      
      /*!\brief Set the command speed
       */
      virtual void Set(float speed);
      
      /*!\brief Get the command speed
       */
      virtual float Get(void);
      
      /*!\brief Enable the SmartDrive
       */
      void Enable(bool bUseSlip = true);
      
      /*!\brief Disable the SmartDrive
       */
      void Disable(void);
      
      /*!\brief Reset the SmartDrive
       */
      void Reset(void);
};

#endif // SMARTDRIVE_H
