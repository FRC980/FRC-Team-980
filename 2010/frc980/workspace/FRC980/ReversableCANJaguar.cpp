#include "ReversableCANJaguar.h"

//==============================================================================
//==============================================================================
ReversableCANJaguar::ReversableCANJaguar(UINT8 deviceNumber,
                                      bool bReversed, ControlMode controlMode)
      
   : CANJaguar(deviceNumber, controlMode)
   , m_bReversed(bReversed)
{
   
}

//==============================================================================
ReversableCANJaguar::~ReversableCANJaguar()
{
   
}

//==============================================================================
//==============================================================================
float ReversableCANJaguar::Get(void)
{
   if (m_bReversed)
      return - CANJaguar::Get();
   return CANJaguar::Get();
}

//==============================================================================
void ReversableCANJaguar::Set(float value)
{
   if (m_bReversed)
      CANJaguar::Set(- value);
   else
      CANJaguar::Set(value);
}

//==============================================================================
bool ReversableCANJaguar::GetReversed(void)
{
   return m_bReversed;
}

//==============================================================================
void ReversableCANJaguar::SetReversed(bool bReversed)
{
   m_bReversed = bReversed;
}
