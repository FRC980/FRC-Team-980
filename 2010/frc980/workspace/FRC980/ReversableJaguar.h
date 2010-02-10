#ifndef REVERSABLEJAGUAR_H
#define REVERSABLEJAGUAR_H

#include <Jaguar.h>

//! A class to reverse Jaguare control on the robot
/*!\class ReversableJaguar
 *
 * The purpose of the ReversableJaguar class is to provide reversable
 * access to the Jaguar speed controllers for the motors
 *    
 */
class ReversableJaguar : public Jaguar
{
   private:
      
      //--- Instance Variables -------------------------------------------------
      bool m_bReversed;
    
   public:
      
      //--- Instance Variables -------------------------------------------------
      
      
      //--- Constructors -------------------------------------------------------
      explicit ReversableJaguar(UINT32 channel, bool bReversed = true);
      
      /*!\brief The SmartDrive destructor
       */
      ReversableJaguar(UINT32 slot, UINT32 channel, bool bReversed = true);
      
      //--- Destructors --------------------------------------------------------
      virtual ~ ReversableJaguar();
      
      //--- Methods ------------------------------------------------------------
      virtual float Get(void);
      
      virtual void Set(float value);
  
      bool GetReversed(void);
      
      void SetReversed(bool bReversed = true);
};

#endif // REVERSABLEJAGUAR_H
