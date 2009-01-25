/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "../WPILib.h"
#include "CJaguar.h"
#include "CWrappers.h"
#include "CPWM.h"

/**
 * Create a Jaguar speed controller object.
 * Allocate the object itself. This is a callback from the CPWM.cpp code to create the
 * actual specific PWM object type.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The PWM channel connected to this speed controller
 */
static SensorBase *CreateJaguar(UINT32 slot, UINT32 channel)
{
	return new Jaguar(slot, channel);
}

/**
 * Set the PWM value.
 *
 * The PWM value is set using a range of -1.0 to 1.0, appropriately
 * scaling the value for the FPGA.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The PWM channel connected to this speed controller
 * @param speed The speed value between -1.0 and 1.0 to set.
 */
void SetJaguarSpeed(UINT32 slot, UINT32 channel, float speed)
{
	Jaguar *jaguar = (Jaguar *) AllocatePWM(slot, channel, CreateJaguar);
	if (jaguar)	jaguar->Set(speed);
}

/**
 * Set the PWM value.
 *
 * The PWM value is set using a range of -1.0 to 1.0, appropriately
 * scaling the value for the FPGA.
 *
 * @param channel The PWM channel connected to this speed controller
 * @param speed The speed value between -1.0 and 1.0 to set.
 */
void SetJaguarSpeed(UINT32 channel, float speed)
{
	Jaguar *jaguar = (Jaguar *) AllocatePWM(SensorBase::GetDefaultDigitalModule(), channel, CreateJaguar);
	if (jaguar) jaguar->Set(speed);
}

/**
 * Set the PWM value directly to the hardware.
 *
 * Write a raw value to a PWM channel.
 *
 * @param channel The PWM channel connected to this speed controller
 * @param value Raw PWM value.  Range 0 - 255.
 */
void SetJaguarRaw(UINT32 channel, UINT8 value)
{
	Jaguar *jaguar = (Jaguar *) AllocatePWM(SensorBase::GetDefaultDigitalModule(), channel, CreateJaguar);
	if (jaguar) jaguar->SetRaw(value);
}

/**
 * Get the PWM value directly from the hardware.
 *
 * Read a raw value from a PWM channel.
 *
 * @param channel The PWM channel connected to this speed controller
 * @return Raw PWM control value.  Range: 0 - 255.
 */
UINT8 GetJaguarRaw(UINT32 channel)
{
	Jaguar *jaguar = (Jaguar *) AllocatePWM(SensorBase::GetDefaultDigitalModule(), channel, CreateJaguar);
	if (jaguar)
		return jaguar->GetRaw();
	else
		return 0;
}

/**
 * Set the PWM value directly to the hardware.
 *
 * Write a raw value to a PWM channel.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The PWM channel connected to this speed controller
 * @param value Raw PWM value.  Range 0 - 255.
 */
void SetJaguarRaw(UINT32 slot, UINT32 channel, UINT8 value)
{
	Jaguar *jaguar = (Jaguar *) AllocatePWM(slot, channel, CreateJaguar);
	if (jaguar) jaguar->SetRaw(value);
}

/**
 * Get the PWM value directly from the hardware.
 *
 * Read a raw value from a PWM channel.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The PWM channel connected to this speed controller
 * @return Raw PWM control value.  Range: 0 - 255.
 */
UINT8 GetJaguarRaw(UINT32 slot, UINT32 channel)
{
	Jaguar *jaguar = (Jaguar *) AllocatePWM(slot, channel, CreateJaguar);
	if (jaguar)
		return jaguar->GetRaw();
	else
		return 0;
}

/**
 * Free the underlying Jaguar object.
 * Free the underlying object and free the associated resources.
 *
 * @param slot The slot the digital module is plugged into
 * @param channel The PWM channel connected to this speed controller
 */
void DeleteJaguar(UINT32 slot, UINT32 channel)
{
	Jaguar *jaguar = (Jaguar *) AllocatePWM(slot, channel, CreateJaguar);
	DeletePWM(slot, channel);
	delete jaguar;
}

/**
 * Free the underlying Jaguar object.
 * Free the underlying object and free the associated resources.
 *
 * @param channel The PWM channel connected to this speed controller
 */
void DeleteJaguar(UINT32 channel)
{
	DeleteJaguar(SensorBase::GetDefaultDigitalModule(), channel);
}

