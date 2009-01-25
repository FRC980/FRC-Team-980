/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Encoder.h"
#include "SensorBase.h"
#include "DigitalModule.h"
#include "CEncoder.h"

static Encoder* encoders[SensorBase::kDigitalModules][SensorBase::kDigitalChannels];
static bool initialized = false;

/**
 * Allocate the resources associated with this encoder.
 * Allocate an Encoder object and cache the value in the associated table to find it
 * in the future.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
static Encoder *AllocateEncoder(UINT32 aSlot, UINT32 aChannel, UINT32 bSlot, UINT32 bChannel)
{
	Encoder *encoder;
	if (!initialized)
	{
		for (unsigned slot = 0; slot < SensorBase::kDigitalModules; slot++)
			for (unsigned channel = 0; channel < SensorBase::kDigitalChannels; channel++)
				encoders[slot][channel] = NULL;
		initialized = true;
	}
	// check if the channel and slots are valid values
	if (!SensorBase::CheckDigitalModule(aSlot)
			|| !SensorBase::CheckDigitalChannel(aChannel)
			|| !SensorBase::CheckDigitalModule(bSlot)
			|| !SensorBase::CheckDigitalChannel(bChannel)) return NULL;
	// check if nothing has been allocated to that pair of channels
	if (encoders[DigitalModule::SlotToIndex(aSlot)][aChannel - 1] == NULL
				&& encoders[DigitalModule::SlotToIndex(bSlot)][bChannel - 1] == NULL)
	{
		// allocate an encoder and put it into both slots
		encoder = new Encoder(aSlot, aChannel, bSlot, bChannel);
		encoders[DigitalModule::SlotToIndex(aSlot)][aChannel - 1] = encoder;
		encoders[DigitalModule::SlotToIndex(bSlot)][bChannel - 1] = encoder;
		return encoder;
	}
	// if there was the same encoder object allocated to both channels, return it
	if ((encoder = encoders[DigitalModule::SlotToIndex(aSlot)][aChannel - 1]) ==
				 encoders[DigitalModule::SlotToIndex(bSlot)][bChannel - 1])
		return encoder;
	// Otherwise, one of the channels is allocated and the other isn't, so this is a
	// resource allocation error.
	return NULL;
}

/**
 * Allocate the resources associated with this encoder.
 * Allocate an Encoder object and cache the value in the associated table to find it
 * in the future.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
static Encoder *AllocateEncoder(UINT32 aChannel, UINT32 bChannel)
{
	return AllocateEncoder(SensorBase::GetDefaultDigitalModule(), aChannel,
							SensorBase::GetDefaultDigitalModule(), bChannel);
}

/**
 * Start the encoder counting.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void StartEncoder(UINT32 aChannel, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
	{
		encoder->Start();
	}
}

/**
 * Start the encoder counting.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void StartEncoder(UINT32 aSlot, UINT32 aChannel, UINT32 bSlot, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aSlot, aChannel, bSlot, bChannel);
	if (encoder != NULL)
	{
		encoder->Start();
	}
}

/**
 * Return the count from the encoder object.
 * Return the count from the encoder. The encoder object returns 4x the number of "ticks"
 * since it counts all four edges.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
INT32 GetEncoder(UINT32 aChannel, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		return encoder->Get();
	else
		return 0;
}

/**
 * Return the count from the encoder object.
 * Return the count from the encoder. The encoder object returns 4x the number of "ticks"
 * since it counts all four edges.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
INT32 GetEncoder(UINT32 aSlot, UINT32 aChannel, UINT32 bSlot, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aSlot, aChannel, bSlot, bChannel);
	if (encoder != NULL)
		return encoder->Get();
	else
		return 0;
}

/**
 * Reset the count for the encoder object.
 * Resets the count to zero.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void ResetEncoder(UINT32 aChannel, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		encoder->Reset();
}

/**
 * Reset the count for the encoder object.
 * Resets the count to zero.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void ResetEncoder(UINT32 aSlot, UINT32 aChannel, UINT32 bSlot, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aSlot, aChannel, bSlot, bChannel);
	if (encoder != NULL)
		encoder->Reset();
}

/**
 * Stops the counting for the encoder object.
 * Stops the counting for the Encoder. It still retains the count, but it doesn't change
 * with pulses until it is started again.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void StopEncoder(UINT32 aChannel, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		encoder->Stop();
}

/**
 * Stops the counting for the encoder object.
 * Stops the counting for the Encoder. It still retains the count, but it doesn't change
 * with pulses until it is started again.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void StopEncoder(UINT32 aSlot, UINT32 aChannel, UINT32 bSlot, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aSlot, aChannel, bSlot, bChannel);
	if (encoder != NULL)
		encoder->Stop();
}

/**
 * Get the encoder period.
 * Return the time between the last two counts in seconds. The value has microsecond accuracy.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @returns The time in seconds between the last two counts.
 */
double GetEncoderPeriod(UINT32 aChannel, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		return encoder->GetPeriod();
	else
		return 0.0;
}

/**
 * Get the encoder period.
 * Return the time between the last two counts in seconds. The value has microsecond accuracy.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @returns The time in seconds between the last two counts.
 */
double GetEncoderPeriod(UINT32 aSlot, UINT32 aChannel, UINT32 bSlot, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aSlot, aChannel, bSlot, bChannel);
	if (encoder != NULL)
		return encoder->GetPeriod();
	else
		return 0.0;
}


/**
 * Sets the maximum period for stopped detection.
 * Sets the value that represents the maximum period of the QuadEncoder before it will assume
 * that the attached device is stopped. This timeout allows users to determine if the wheels or
 * other shaft has stopped rotating.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @param maxPeriod The maximum time between rising and falling edges before the FPGA will
 * consider the device stopped. This is expressed in seconds.
 */
void SetMaxEncoderPeriod(UINT32 aChannel, UINT32 bChannel, UINT32 maxPeriod)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		encoder->SetMaxPeriod(maxPeriod);
}

/**
 * Sets the maximum period for stopped detection.
 * Sets the value that represents the maximum period of the QuadEncoder before it will assume
 * that the attached device is stopped. This timeout allows users to determine if the wheels or
 * other shaft has stopped rotating.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @param maxPeriod The maximum time between rising and falling edges before the FPGA will
 * consider the device stopped. This is expressed in seconds.
 */
void SetMaxEncoderPeriod(UINT32 aSlot, UINT32 aChannel, UINT32 bSlot, UINT32 bChannel, UINT32 maxPeriod)
{
	Encoder *encoder = AllocateEncoder(aSlot, aChannel, bSlot, bChannel);
	if (encoder != NULL)
		encoder->SetMaxPeriod(maxPeriod);
}

/**
 * Determine if the encoder is stopped.
 * Using the MaxPeriod value, a boolean is returned that is true if the encoder is considered
 * stopped and false if it is still moving. A stopped encoder is one where the most recent pulse
 * width exceeds the MaxPeriod.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @return True if the encoder is considered stopped.
 */
bool GetEncoderStopped(UINT32 aChannel, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		return encoder->GetStopped();
	else
		return false;
}

/**
 * Determine if the encoder is stopped.
 * Using the MaxPeriod value, a boolean is returned that is true if the encoder is considered
 * stopped and false if it is still moving. A stopped encoder is one where the most recent pulse
 * width exceeds the MaxPeriod.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @return True if the encoder is considered stopped.
 */
bool GetEncoderStopped(UINT32 aSlot, UINT32 aChannel, UINT32 bSlot, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aSlot, aChannel, bSlot, bChannel);
	if (encoder != NULL)
		return encoder->GetStopped();
	else
		return false;
}

/**
 * The last direction the encoder value changed.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @return The last direction the encoder value changed.
 */
bool GetEncoderDirection(UINT32 aChannel, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		return encoder->GetDirection();
	else
		return false;
}

/**
 * The last direction the encoder value changed.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @return The last direction the encoder value changed.
 */
bool GetEncoderDirection(UINT32 aSlot, UINT32 aChannel, UINT32 bSlot, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aSlot, aChannel, bSlot, bChannel);
	if (encoder != NULL)
		return encoder->GetDirection();
	else
		return false;
}

/**
 * Get the distance the robot has driven since the last reset
 * @return The distance driven since the last reset based on the distance per tick
 * variable being set by SetDistancePerTick(). It is just a simple multiplication, but
 * makes the bookkeeping a little easier since the encoder remembers the scale factor.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @returns The distance traveled based on the distance per tick.
 */
float GetEncoderDistance(UINT32 aChannel, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		return encoder->GetDistance();
	else
		return 0.0;
}

/**
 * Get the distance the robot has driven since the last reset
 * @return The distance driven since the last reset based on the distance per tick
 * variable being set by SetDistancePerTick(). It is just a simple multiplication, but
 * makes the bookkeeping a little easier since the encoder remembers the scale factor.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
  */
float GetEncoderDistance(UINT32 aSlot, UINT32 aChannel, UINT32 bSlot, UINT32 bChannel)
{
	Encoder *encoder = AllocateEncoder(aSlot, aChannel, bSlot, bChannel);
	if (encoder != NULL)
		return encoder->GetDistance();
	else
		return 0.0;
}

/**
 * Set the distance per tick for this encoder.
 * This sets the multiplier used to determine the distance driven based on the count value
 * from the encoder. Reseting the encoder also resets the distance since it's just a simple
 * multiply.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @param distancePerTick The distance traveled per tick of the encoder
 */
void SetEncoderDistancePerTick(UINT32 aChannel, UINT32 bChannel, float distancePerTick)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		encoder->SetDistancePerTick(distancePerTick);
}

/**
 * Set the distance per tick for this encoder.
 * This sets the multiplier used to determine the distance driven based on the count value
 * from the encoder. Reseting the encoder also resets the distance since it's just a simple
 * multiply.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @param distancePerTick The multiplier used to return the distance traveled
 */
void SetEncoderDistancePerTick(UINT32 aSlot, UINT32 aChannel, UINT32 bSlot, UINT32 bChannel, float distancePerTick)
{
	Encoder *encoder = AllocateEncoder(aSlot, aChannel, bSlot, bChannel);
	if (encoder != NULL)
		encoder->SetDistancePerTick(distancePerTick);
}

/**
 * Set the direction sensing for this encoder.
 * This sets the direction sensing on the encoder so that it could count in the correct
 * software direction regardless of the mounting.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @param reverseDirection true if the encoder direction should be reversed
 */
void SetEncoderReverseDirection(UINT32 aChannel, UINT32 bChannel, bool reverseDirection)
{
	Encoder *encoder = AllocateEncoder(aChannel, bChannel);
	if (encoder != NULL)
		encoder->SetReverseDirection(reverseDirection);
}

/**
 * Set the direction sensing for this encoder.
 * This sets the direction sensing on the encoder so that it couldl count in the correct
 * software direction regardless of the mounting.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 * @param reverseDirection true if the encoder direction should be reversed
 */
void SetEncoderReverseDirection(UINT32 aSlot, UINT32 aChannel, UINT32 bSlot, UINT32 bChannel, bool reverseDirection)
{
	Encoder *encoder = AllocateEncoder(aSlot, aChannel, bSlot, bChannel);
	if (encoder != NULL)
		encoder->SetReverseDirection(reverseDirection);
}

/**
 * Free the resources associated with this encoder.
 * Delete the Encoder object and the entries from the cache for this encoder.
 *
 * @param aSlot The digital module slot for the A Channel on the encoder
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bSlot The digital module slot for the B Channel on the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void DeleteEncoder(UINT32 aSlot, UINT32 aChannel, UINT32 bSlot, UINT32 bChannel)
{
	if (SensorBase::CheckDigitalModule(aSlot) && SensorBase::CheckDigitalChannel(aChannel))
	{
		delete encoders[DigitalModule::SlotToIndex(aSlot)][aChannel - 1];
		encoders[DigitalModule::SlotToIndex(aSlot)][aChannel - 1] = NULL;
	}
}

/**
 * Free the resources associated with this encoder.
 * Delete the Encoder object and the entries from the cache for this encoder.
 *
 * @param aChannel The channel on the digital module for the A Channel of the encoder
 * @param bChannel The channel on the digital module for the B Channel of the encoder
 */
void DeleteEncoder(UINT32 aChannel, UINT32 bChannel)
{
	DeleteEncoder(SensorBase::GetDefaultDigitalModule(), aChannel, SensorBase::GetDefaultDigitalModule(), bChannel);
}

