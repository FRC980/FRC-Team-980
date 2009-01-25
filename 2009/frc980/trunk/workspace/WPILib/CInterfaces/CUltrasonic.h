/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/
#ifndef C_ULTRASONIC_H
#define C_ULTRASONIC_H

#include "Ultrasonic.h"

void InitUltrasonic(UINT32 pingChannel, UINT32 echoChannel);
void InitUltrasonic(UINT32 pingSlot, UINT32 pingChannel, UINT32 echoSlot, UINT32 echoChannel);
double GetUltrasonicInches(UINT32 pingChannel, UINT32 echoChannel);
double GetUltrasonicInches(UINT32 pingSlot, UINT32 pingChannel, UINT32 echoSlot, UINT32 echoChannel);
double GetUltrasonicMM(UINT32 pingChannel, UINT32 echoChannel);
double GetUltrasonicMM(UINT32 pingSlot, UINT32 pingChannel, UINT32 echoSlot, UINT32 echoChannel);
void DeleteUltrasonic(UINT32 pingChannel, UINT32 echoChannel);
void DeleteUltrasonic(UINT32 pingSlot, UINT32 pingChannel, UINT32 echoSlot, UINT32 echoChannel);

#endif
