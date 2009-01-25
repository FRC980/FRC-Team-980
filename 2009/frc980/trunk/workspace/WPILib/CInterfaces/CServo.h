/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef C_SERVO_H
#define C_SERVO_H

void SetServo(UINT32 slot, UINT32 channel, float value);
float GetGetServo(UINT32 slot, UINT32 channel);
void SetServoAngle(UINT32 slot, UINT32 channel, float angle);
float GetServoAngle(UINT32 slot, UINT32 channel);
float GetServoMaxAngle(UINT32 slot, UINT32 channel);
float GetServoMinAngle(UINT32 slot, UINT32 channel);
void SetServo(UINT32 channel, float value);
float GetGetServo(UINT32 channel);
void SetServoAngle(UINT32 channel, float angle);
float GetServoAngle(UINT32 channel);
float GetServoMaxAngle(UINT32 channel);
float GetServoMinAngle(UINT32 channel);
void DeleteServo(UINT32 slot, UINT32 channel);
void DeleteServo(UINT32 channel);

#endif

