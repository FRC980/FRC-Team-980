/*---------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                            */
/* Open Source Software - may be modified and shared by FRC teams. The code  */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib. */
/*---------------------------------------------------------------------------*/

#include "PIDController.h"
#include "Notifier.h"
#include "PIDSource.h"
#include "PIDOutput.h"
#include <math.h>

/**
 * Allocate a PID object with the given constants for P, I, D
 * @param Kp the proportional coefficient
 * @param Ki the integral coefficient
 * @param Kd the derivative coefficient
 * @param period the loop time for doing calculations. This particularly
 * effects calculations of the integral and differental terms. The default
 * is 50ms.
 */
PIDController::PIDController(float Kp, float Ki, float Kd, float period)
{
    m_controlLoop = new Notifier(PIDController::CallCalculate, this);

    m_P = Kp;
    m_I = Ki;
    m_D = Kd;
    m_maximumOutput = 1.0;
    m_minimumOutput = -1.0;

    m_maximumInput = 0;
    m_minimumInput = 0;

    m_continuous = false;
    m_enabled = false;
    m_setpoint = 0;

    m_prevError = 0;
    m_totalError = 0;
    m_tolerence = .05;

    m_result = 0;

    m_pidInput = 0;
    m_pidOutput = 0;

    m_controlLoop->StartPeriodic(period);
}

/**
 * Free the PID object
 */
PIDController::~PIDController()
{
    delete m_controlLoop;
}

/**
 * Call the Calculate method as a non-static method. This avoids having to
 * prepend all local variables in that method with the class pointer. This
 * way the "this" pointer will be set up and class variables can be called
 * more easily.  This method is static and called by the Notifier class.
 * @param controller the address of the PID controller object to use in
 * the background loop
 */
void PIDController::CallCalculate(void *controller)
{
    PIDController *control = (PIDController*) controller;
    control->Calculate();
}

/**
 * Read the input, calculate the output accordingly, and write to the
 * output.  This should only be called by the Notifier indirectly through
 * CallCalculate and is created during initialization.
 */
void PIDController::Calculate()
{
    if (m_pidInput == 0)
        return;
    if (m_pidOutput == 0)
        return;
    float input = m_pidInput->PIDGet();

    if (m_enabled)
    {
        m_error = m_setpoint - input;
        if (m_continuous)
        {
            if (fabs(m_error) >
                m_maximumInput - m_minimumInput)
            {
                if (m_error > 0)
                    m_error = m_error  - m_maximumInput + m_minimumInput;
                else
                    m_error = m_error  +
                    m_maximumInput - m_minimumInput;
            }
        }

        if (((m_totalError + m_error) * m_I < m_maximumOutput) &&
                ((m_totalError + m_error) * m_I > m_minimumOutput))
            m_totalError += m_error;

        m_result = m_P * m_error + m_I * m_totalError +
            m_D * (m_error - m_prevError);
        m_prevError = m_error;

        if (m_result > m_maximumOutput)
            m_result = m_maximumOutput;
        else if (m_result < m_minimumOutput)
            m_result = m_minimumOutput;

        m_pidOutput->PIDWrite(m_result);
    }
}

/**
 * Return the current PID result
 * This is always centered on zero and constrained the the max and min outs
 * @return the latest calculated output
 */
float PIDController::Get()
{
    return m_result;
}

/**
 * Set the PID controller to consider the input to be continuous, Rather
 * then using the max and min in as constraints, it considers them to be
 * the same point and automatically calculates the shortest route to the
 * setpoint.
 * @param continuous Set to true turns on continuous, false turns off
 * continuous
 */
void PIDController::SetContinuous(bool continuous)
{
    m_continuous = continuous;
}

/**
 * Sets the PIDSource object from which the PIDController gets its
 * feedback.
 * @param pidInput the source of feedback for the PIDController
 */
void PIDController::SetInput(PIDSource *pidInput)
{
    m_pidInput = pidInput;
}

/**
 * Sets the PIDSource object from which the PIDController gets its
 * feedback, as well as the maximum and minimum values expected from the
 * input.
 * @param pidInput the source of feedback for the PIDController
 * @param minimumInput the minimum value expected from the input
 * @param maximumInput the maximum value expected from the output
 */
void PIDController::SetInput(PIDSource *pidInput, float minimumInput,
        float maximumInput)
{
    m_pidInput = pidInput;
    m_minimumInput = minimumInput;
    m_maximumInput = maximumInput;
    SetSetpoint(m_setpoint);
}

/**
 * Sets the PIDOutput object which the PIDController writes to.
 * @param pidOutput the source of feedback for the PIDController
 */
void PIDController::SetOutput(PIDOutput *pidOutput)
{
    m_pidOutput = pidOutput;
}

/**
 * Sets the PIDOutput object which the PIDController writes to, as well as
 * the minimum and maximum values to write.
 * @param pidOutput the source of feedback for the PIDController
 * @param minimumOutput the minimum value to write to the output
 * @param maximumOutput the maximum value to write to the output
 */
void PIDController::SetOutput(PIDOutput *pidOutput, float minimumOutput,
        float maximumOutput)
{
    m_pidOutput = pidOutput;
    m_minimumOutput = minimumOutput;
    m_maximumOutput = maximumOutput;
}

/**
 * Set the setpoint for the PIDController
 * @param setpoint the desired setpoint
 */
void PIDController::SetSetpoint(float setpoint)
{
    if (m_maximumInput > m_minimumInput) {
        if (setpoint > m_maximumInput)
            m_setpoint = m_maximumInput;
        else if (setpoint < m_minimumInput)
            m_setpoint = m_minimumInput;
        else
            m_setpoint = setpoint;
    }
    else
        m_setpoint = setpoint;
}

/**
 * Returns the current setpoint of the PIDController
 * @return the current setpoint
 */
float PIDController::GetSetpoint()
{
    return m_setpoint;
}

/**
 * Retruns the current difference of the input from the setpoint
 * @return the current error
 */
float PIDController::GetError()
{
    return m_error;
}

/*
 * Set the percentage error which is considered tolerable for use with
 * OnTarget.
 * @param percentage error which is tolerable
 */
void PIDController::SetTolerence(float percent)
{
    m_tolerence = percent;
}

/*
 * Return true if the error is within the percentage of the total input
 * range, determined by SetTolerance. This asssumes that the maximum and
 * minimum input were set using SetInput.
 */
bool PIDController::OnTarget()
{
    return (fabs(m_error)<m_tolerence / 100 *
            (m_maximumInput - m_minimumInput));
}

/**
 * Begin running the PIDController
 */
void PIDController::Enable()
{
    m_enabled = true;
}

/**
 * Stop running the PIDController, this sets the output to zero before
 * stopping.
 */
void PIDController::Disable()
{
    m_enabled = false;
    m_pidOutput->PIDWrite(0);
}

/**
 * Reset the previous error,, the integral term, and disable the
 * controller.
 */
void PIDController::Reset()
{
    Disable();
    m_prevError = 0;
    m_totalError = 0;
    m_result = 0;
}
