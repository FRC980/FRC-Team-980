#include <stdio.h>
#include "HOT_PID.h"

/***
 * Allocate a PID object with the given constants for Kp, Ki and Kd
 */
HOT_PID::HOT_PID(float Kp, float Ki, float Kd, float tolerance)
{
    m_Kp = Kp;
    m_Ki = Ki;
    m_Kd = Kd;

    m_tolerance = tolerance;

    m_error_p = 0.0;
    m_error_i = 0.0;
    m_error_d = 0.0;

    m_SP_max = +1.0;
    m_SP_min = -1.0;

    m_PV_max = +1.0;
    m_PV_min = -1.0;

    m_MV_max = +1.0;
    m_MV_min = -1.0;

    m_enabled = false;
}

/**
 * Free the PID object
 */
HOT_PID::~HOT_PID(void)
{
}

// Calculate the Manipulated Variable
float HOT_PID::GetMV(float SP, float PV)
{
    if (m_enabled)
    {
        // limit the Set Point
        if (SP > m_SP_max)
        {
            SP = m_SP_max;
        }
        else if (SP < m_SP_min)
        {
            SP = m_SP_min;
        }

        // limit the Process Variable
        if (PV > m_PV_max)
        {
            PV = m_PV_max;
        }
        else if (PV < m_PV_min)
        {
            PV = m_PV_min;
        }
        // find the errors
        float error_p;
        float error_i;
        float error_d;

        // proportional error is difference between Set Point and Process
        // Variable
        error_p = SP - PV;

        // integral error is sum of current and previous proportional
        // errors
        if ((error_p * m_error_i) < 0.0) // zero the sum if sign has changed
        {
            error_i = 0.0;
        }
        // continue to sum if not yet over the output limit
        else if (((m_error_i + error_p) * m_Ki < m_MV_max)
                 && ((m_error_i + error_p) * m_Ki > m_MV_min))
        {
            error_i = m_error_i + error_p;
        }
        // error remains the same
        else
        {
            error_i = m_error_i;
        }

        // derivative error is difference between current and last
        // proportional errors
        error_d = error_p - m_error_p;

        // save the errors
        m_error_p = error_p;
        m_error_i = error_i;
        m_error_d = error_d;

        // Find the Manipulated Variable
        float MV = m_Kp * error_p + m_Ki * error_i + m_Kd * error_d;

        //      printf("error_p = %f, error_i = %f,  error_d = %f, MV = %f\n",error_p,error_i,error_d, MV);

        // Limit the Manipulated Variable
        if (MV > m_MV_max)
        {
            MV = m_MV_max;
        }
        else if (MV < m_MV_min)
        {
            MV = m_MV_min;
        }
        // return the Manipulated Variable
        return MV;
    }
    else                        // return 0 if disabled
    {
        return 0.0;
    }
}

// Set the gains
void HOT_PID::SetGains(float Kp, float Ki, float Kd)
{
    m_Kp = Kp;
    m_Ki = Ki;
    m_Kd = Kd;
}

// Set the SetPoint limits
void HOT_PID::SetSPLimits(float max, float min)
{
    m_SP_max = max;
    m_SP_min = min;
}

// Set the Process Variable limits
void HOT_PID::SetPVLimits(float max, float min)
{
    m_PV_max = max;
    m_PV_min = min;
}

// Set the Manipulated Variable Limits
void HOT_PID::SetMVLimits(float max, float min)
{
    m_MV_max = max;
    m_MV_min = min;
}

// Test to see if convergence is complete
bool HOT_PID::OnTarget()
{
    if ((m_error_p < m_tolerance * m_SP_max)
        && (m_error_p > m_tolerance * m_SP_min) &&
        //                      (m_error_i < m_tolerance*m_SP_max) && (m_error_i > m_tolerance*m_SP_min) &&
        (m_error_d < m_tolerance * m_SP_max)
        && (m_error_d > m_tolerance * m_SP_min))
    {
        return true;
    }
    else
    {
        return false;
    }
}

// Reset PID by clearing the error variables
void HOT_PID::Reset()
{
    m_error_p = 0.0;
    m_error_i = 0.0;
    m_error_d = 0.0;
}

// Enable control
void HOT_PID::Enable()
{
    m_enabled = true;
}

// Enable control
void HOT_PID::Disable()
{
    m_enabled = false;
}
