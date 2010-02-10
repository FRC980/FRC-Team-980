#include "ReversableJaguar.h"

//==============================================================================
//==============================================================================
ReversableJaguar::ReversableJaguar(UINT32 channel,
                                   bool bReversed)
   : Jaguar(channel)
   , m_bReversed(bReversed)
{
   
}

//==============================================================================
ReversableJaguar::ReversableJaguar(UINT32 slot,
                                   UINT32 channel,
                                   bool bReversed)
   : Jaguar(slot, channel)
   , m_bReversed(bReversed)
{
   
}

//==============================================================================
ReversableJaguar::~ReversableJaguar()
{
   
}

//==============================================================================
//==============================================================================
float ReversableJaguar::Get(void)
{
   if (m_bReversed)
      return - Jaguar::Get();
   return Jaguar::Get();
}

//==============================================================================
void ReversableJaguar::Set(float value)
{
   if (m_bReversed)
      Jaguar::Set(- value);
   else
      Jaguar::Set(value);
}

//==============================================================================
bool ReversableJaguar::GetReversed(void)
{
   return m_bReversed;
}

//==============================================================================
void ReversableJaguar::SetReversed(bool bReversed = true)
{
   m_bReversed = bReversed;
}
