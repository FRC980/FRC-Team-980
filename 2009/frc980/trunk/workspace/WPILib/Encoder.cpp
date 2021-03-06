/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Encoder.h"
#include "DigitalInput.h"
#include "Resource.h"
#include "Utility.h"
#include "WPIStatus.h"

static Resource *quadEncoders = NULL;

/**
 * Common initialization code for QuadEncoders.
 * This code allocates resources for QuadEncoders and is common to all constructors.
 */
void Encoder::InitEncoder(bool reverseDirection)
{
    Resource::CreateResourceObject(&quadEncoders, tEncoder::kNumSystems);
    //TODO: need to check for errors here
    m_index = quadEncoders->Allocate();
    m_encoder = new tEncoder(m_index, &status);
    m_encoder->writeConfig_ASource_Module(m_aSource->GetModuleForRouting(),
                                          &status);
    m_encoder->writeConfig_ASource_Channel(m_aSource->
                                           GetChannelForRouting(), &status);
    m_encoder->writeConfig_ASource_AnalogTrigger(m_aSource->
                                                 GetAnalogTriggerForRouting
                                                 (), &status);
    m_encoder->writeConfig_BSource_Module(m_bSource->GetModuleForRouting(),
                                          &status);
    m_encoder->writeConfig_BSource_Channel(m_bSource->
                                           GetChannelForRouting(), &status);
    m_encoder->writeConfig_BSource_AnalogTrigger(m_bSource->
                                                 GetAnalogTriggerForRouting
                                                 (), &status);
    m_encoder->strobeReset(&status);
    m_encoder->writeConfig_Reverse(reverseDirection, &status);
    m_distancePerTick = 1.0;
    wpi_assertCleanStatus(status);
}

/**
 * QuadEncoder constructor.
 * Construct a QuadEncoder given a and b modules and channels fully specified.
 * @param aSlot The a channel digital input module.
 * @param aChannel The a channel digital input channel.
 * @param bSlot The b channel digital input module.
 * @param bChannel The b channel digital input channel.
 * @param reverseDirection represents the orientation of the encoder and inverts the output values
 * if necessary so forward represents positive values.
 */
Encoder::Encoder(UINT32 aSlot, UINT32 aChannel,
                 UINT32 bSlot, UINT32 bChannel, bool reverseDirection)
{
    m_aSource = new DigitalInput(aSlot, aChannel);
    m_bSource = new DigitalInput(bSlot, bChannel);
    InitEncoder(reverseDirection);
    m_allocatedASource = true;
    m_allocatedBSource = true;
}

/**
 * QuadEncoder constructor.
 * Construct a QuadEncoder given a and b channels assuming the default module.
 * @param aChannel The a channel digital input channel.
 * @param bChannel The b channel digital input channel.
 * @param reverseDirection represents the orientation of the encoder and inverts the output values
 * if necessary so forward represents positive values.
 */
Encoder::Encoder(UINT32 aChannel, UINT32 bChannel, bool reverseDirection)
{
    m_aSource = new DigitalInput(aChannel);
    m_bSource = new DigitalInput(bChannel);
    InitEncoder(reverseDirection);
    m_allocatedASource = true;
    m_allocatedBSource = true;
}

/**
 * QuadEncoder constructor.
 * Construct a QuadEncoder given a and b channels as digital inputs. This is used in the case
 * where the digital inputs are shared. The QuadEncoder class will not allocate the digital inputs
 * and assume that they already are counted.
 * @param aSource The source that should be used for the a channel.
 * @param bSource the source that should be used for the b channel.
 * @param reverseDirection represents the orientation of the encoder and inverts the output values
 * if necessary so forward represents positive values.
 */
Encoder::Encoder(DigitalSource * aSource, DigitalSource * bSource,
                 bool reverseDirection)
{
    m_aSource = aSource;
    m_bSource = bSource;
    m_allocatedASource = false;
    m_allocatedBSource = false;
    if (m_aSource == NULL || m_bSource == NULL)
        wpi_fatal(NullParameter);
    else
        InitEncoder(reverseDirection);
}

Encoder::Encoder(DigitalSource & aSource, DigitalSource & bSource,
                 bool reverseDirection)
{
    m_aSource = &aSource;
    m_bSource = &bSource;
    m_allocatedASource = false;
    m_allocatedBSource = false;
    InitEncoder(reverseDirection);
}

/**
 * Free the resources for a QuadEncoder.
 * Frees the FPGA resources associated with a Quad Encoder.
 */
Encoder::~Encoder()
{
    if (m_allocatedASource)
        delete m_aSource;
    if (m_allocatedBSource)
        delete m_bSource;
    quadEncoders->Free(m_index);
    delete m_encoder;
}

/**
 * Start the QuadEncoder.
 * Starts counting pulses on the QuadEncoder device.
 */
void Encoder::Start()
{
    m_encoder->writeEnable(1, &status);
    wpi_assertCleanStatus(status);
}

/**
 * Stops counting pulses on the QuadEncoder device. The value is not changed.
 */
void Encoder::Stop()
{
    m_encoder->writeEnable(0, &status);
    wpi_assertCleanStatus(status);
}

/**
 * Gets the current count.
 * Returns the current count on the QuadEncoder. Quadrature encoders return 4x the
 * expected number of counts since the hardware counts all 4 edges for higher
 * resolution.
 *
 * @return Current count from the QuadEncoder.
 */
INT32 Encoder::Get()
{
    INT32 value = m_encoder->readOutput_Value(&status);
    wpi_assertCleanStatus(status);
    return value;
}

/**
 * Reset the QuadEncoder to zero.
 * Resets the current count to zero on the encoder.
 */
void Encoder::Reset()
{
    m_encoder->strobeReset(&status);
    wpi_assertCleanStatus(status);
}

/**
 * Returns the period of the most recent pulse.
 * Returns the period of the most recent Quad Encoder pulse in seconds.
 * @return Period in seconds of the most recent pulse.
 */
double Encoder::GetPeriod()
{
    UINT32 value = m_encoder->readTimerOutput_Period(&status);
    wpi_assertCleanStatus(status);
    return value / 1.0e6;
}

/**
 * Sets the maximum period for stopped detection.
 * Sets the value that represents the maximum period of the QuadEncoder before it will assume
 * that the attached device is stopped. This timeout allows users to determine if the wheels or
 * other shaft has stopped rotating.
 * @param maxPeriod The maximum time between rising and falling edges before the FPGA will
 * consider the device stopped. This is expressed in seconds.
 */
void Encoder::SetMaxPeriod(double maxPeriod)
{
    m_encoder->writeTimerConfig_StallPeriod((UINT32) (maxPeriod * 1.0e6),
                                            &status);
    wpi_assertCleanStatus(status);
}

/**
 * Determine if the encoder is stopped.
 * Using the MaxPeriod value, a boolean is returned that is true if the encoder is considered
 * stopped and false if it is still moving. A stopped encoder is one where the most recent pulse
 * width exceeds the MaxPeriod.
 * @return True if the encoder is considered stopped.
 */
bool Encoder::GetStopped()
{
    bool value = m_encoder->readTimerOutput_Stalled(&status) != 0;
    wpi_assertCleanStatus(status);
    return value;
}

/**
 * The last direction the encoder value changed.
 * @return The last direction the encoder value changed.
 */
bool Encoder::GetDirection()
{
    bool value = m_encoder->readOutput_Direction(&status);
    wpi_assertCleanStatus(status);
    return value;
}

/**
 * Get the distance the robot has driven since the last reset
 * @return The distance driven since the last reset based on the distance per tick
 * variable being set by SetDistancePerTick(). It is just a simple multiplication, but
 * makes the bookkeeping a little easier since the encoder remembers the scale factor.
 */
float Encoder::GetDistance()
{
    return Get() * m_distancePerTick;
}

/**
 * Set the distance per tick for this encoder.
 * This sets the multiplier used to determine the distance driven based on the count value
 * from the encoder. Reseting the encoder also resets the distance since it's just a simple
 * multiply.
 */
void Encoder::SetDistancePerTick(float distancePerTick)
{
    m_distancePerTick = distancePerTick;
}

/**
 * Set the direction sensing for this encoder.
 * This sets the direction sensing on the encoder so that it couldl count in the correct
 * software direction regardless of the mounting.
 * @param reverseDirection true if the encoder direction should be reversed
 */
void Encoder::SetReverseDirection(bool reverseDirection)
{
    m_encoder->writeConfig_Reverse(reverseDirection, &status);
    wpi_assertCleanStatus(status);
}
