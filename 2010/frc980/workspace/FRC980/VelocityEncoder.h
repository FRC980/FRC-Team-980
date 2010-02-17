#ifndef VELOCITYENCODER_H
#define VELOCITYENCODER_H

#include <PIDSource.h>

//==============================================================================
class Encoder;

//==============================================================================
//! The Velocity Encoder and PID Source for the Robot
/*!\class VelocityEncoder
 *
 * This provides the PIDSource interface in the form of an encoder which
 * reports velocity (as opposed to a normal encoder which provides the
 * PIDSource interface reporting position)
 */
class VelocityEncoder : public PIDSource
{
   private:
      //--- Instance Variables -------------------------------------------------
      Encoder* m_pEnc;
    
   public:
      //--- Instance Variables -------------------------------------------------
      
      //--- Constructors -------------------------------------------------------
      /*!\brief The VelocityEncoder Constructor
       */
      VelocityEncoder(Encoder* pEnc);
      
      //--- Methods ------------------------------------------------------------
      /*!\brief Return the Encoder Velocity/Rate
       */
      virtual double PIDGet(void);
};

#endif // VELOCITYENCODER_H
