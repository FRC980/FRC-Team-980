/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef _C_ENCODER_H
#define _C_ENCODER_H

void StartEncoder(UINT32 aChannel, UINT32 bChannel);
INT32 GetEncoder(UINT32 aChannel, UINT32 bChannel);
void ResetEncoder(UINT32 aChannel, UINT32 bChannel);
void StopEncoder(UINT32 aChannel, UINT32 bChannel);
double GetEncoderPeriod(UINT32 aChannel, UINT32 bChannel);
void SetMaxEncoderPeriod(UINT32 aChannel, UINT32 bChannel, UINT32 maxPeriod);
bool GetEncoderStopped(UINT32 aChannel, UINT32 bChannel);
bool GetEncoderDirection(UINT32 aChannel, UINT32 bChannel);
float GetEncoderDistance(UINT32 aChannel, UINT32 bChannel);
void SetEncoderDistancePerTick(UINT32 aChannel, UINT32 bChannel, float distancePerTick);
void SetEncoderReverseDirection(UINT32 aChannel, UINT32 bChannel, bool reversedDirection);
void StartEncoder(UINT32 aslot, UINT32 aChannel, UINT32 bslot, UINT32 bChannel);
INT32 GetEncoder(UINT32 aslot, UINT32 aChannel, UINT32 bslot, UINT32 bChannel);
void ResetEncoder(UINT32 aslot, UINT32 aChannel, UINT32 bslot, UINT32 bChannel);
void StopEncoder(UINT32 aslot, UINT32 aChannel, UINT32 bslot, UINT32 bChannel);
double GetEncoderPeriod(UINT32 aslot, UINT32 aChannel, UINT32 bslot, UINT32 bChannel);
void SetMaxEncoderPeriod(UINT32 aslot, UINT32 aChannel, UINT32 bslot, UINT32 bChannel, UINT32 maxPeriod);
bool GetEncoderStopped(UINT32 aslot, UINT32 aChannel, UINT32 bslot, UINT32 bChannel);
bool GetEncoderDirection(UINT32 aslot, UINT32 aChannel, UINT32 bslot, UINT32 bChannel);
float GetEncoderDistance(UINT32 aslot, UINT32 aChannel, UINT32 bslot, UINT32 bChannel);
void SetEncoderDistancePerTick(UINT32 aslot, UINT32 aChannel, UINT32 bslot, UINT32 bChannel, float distancePerTick);
void SetEncoderReverseDirection(UINT32 aslot, UINT32 aChannel, UINT32 bslot, UINT32 bChannel, bool reversedDirection);
void DeleteEncoder(UINT32 aChannel, UINT32 bChannel);
void DeleteEncoder(UINT32 aSlot, UINT32 aChannel, UINT32 bSlot, UINT32 bChannel);

#endif

