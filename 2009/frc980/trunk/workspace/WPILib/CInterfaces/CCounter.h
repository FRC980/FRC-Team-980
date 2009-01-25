#ifndef _C_COUNTER_H
#define _C_COUNTER_H

void StartCounter(UINT32 channel);
void StartCounter(UINT32 slot, UINT32 channel);
INT32 GetCounter(UINT32 channel);
INT32 GetCounter(UINT32 slot, UINT32 channel);
void ResetCounter(UINT32 channel);
void ResetCounter(UINT32 slot, UINT32 channel);
void StopCounter(UINT32 channel);
void StopCounter(UINT32 slot, UINT32 channel);
double GetCounterPeriod(UINT32 channel);
double GetCounterPeriod(UINT32 slot, UINT32 channel);
void DeleteCounter(UINT32 slot, UINT32 channel);
void DeleteCounter(UINT32 channel);

#endif
