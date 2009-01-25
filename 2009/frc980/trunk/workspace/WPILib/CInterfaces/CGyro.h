/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef C_GYRO_H
#define C_GYRO_H

#include <VxWorks.h>

void InitGyro(UINT32 slot, UINT32 channel);
void InitGyro(UINT32 channel);
float GetGyroAngle(UINT32 channel);
float GetGyroAngle(UINT32 slot, UINT32 channel);
void ResetGyro(UINT32 channel);
void ResetGyro(UINT32 slot, UINT32 channel);
void SetGyroSensitivity(UINT32 slot, UINT32 channel, float voltsPerDegreePerSecond);
void SetGyroSensitivity(UINT32 channel, float voltsPerDegreePerSecond);
void DeleteGyro(UINT32 slot, UINT32 channel);
void DeleteGyro(UINT32 channel);

#endif

