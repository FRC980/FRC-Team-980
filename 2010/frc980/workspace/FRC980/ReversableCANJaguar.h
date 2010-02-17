#ifndef REVERSABLECANJAGUAR_H
#define REVERSABLECANJAGUAR_H

#include "CANJaguar.h"

//! A class to reverse CAN Jaguar control on the robot
/*!\class ReversableCANJaguar
 *
 * The purpose of the ReversableCANJaguar class is to provide reversable
 * access to the Jaguar speed controllers for the motors
 *    
 */
class ReversableCANJaguar : public CANJaguar
{
   private:
      
      //--- Instance Variables -------------------------------------------------
      bool m_bReversed;
    
   public:
      
      //--- Instance Variables -------------------------------------------------
      
      
      //--- Constructors -------------------------------------------------------     
      /*!\brief The ReversableCANJaguar constructor
       */
      ReversableCANJaguar(UINT8 deviceNumber, bool bReversed = true, ControlMode controlMode = kPercentVoltage);
      
      //--- Destructors --------------------------------------------------------
      virtual ~ ReversableCANJaguar();
      
      //--- Methods ------------------------------------------------------------
      virtual float Get(void);
      
      virtual void Set(float value);
  
      bool GetReversed(void);
      
      void SetReversed(bool bReversed = true);
};

#endif // REVERSABLECANJAGUAR_H
