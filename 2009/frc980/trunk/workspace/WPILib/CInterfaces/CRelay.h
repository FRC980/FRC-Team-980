#ifndef _C_RELAY_H
#define _C_RELAY_H

typedef enum {kOff, kOn, kForward, kReverse} RelayValue;
typedef enum {kBothDirections, kForwardOnly, kReverseOnly} RelayDirection;

void InitRelay(UINT32 channel, RelayDirection direction = kBothDirections);
void InitRelayRelay(UINT32 slot, UINT32 channel, RelayDirection direction = kBothDirections);

void DeleteRelay(UINT32 channel);
void DeleteRelay(UINT32 slot, UINT32 channel);

void SetRelay(UINT32 channel, RelayValue value);
void SetRelay(UINT32 slot, UINT32 channel, RelayValue value);

#endif

