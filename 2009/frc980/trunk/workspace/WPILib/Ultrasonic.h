/*---------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                            */
/* Open Source Software - may be modified and shared by FRC teams. The code  */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib. */
/*---------------------------------------------------------------------------*/

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include "SensorBase.h"
#include "Task.h"

class Counter;
class DigitalInput;
class DigitalOutput;

/**
 * Ultrasonic rangefinder class.
 * The Ultrasonic rangefinder measures absolute distance based on the
 * round-trip time of a ping generated by the controller. These sensors
 * use two transducers, a speaker and a microphone both tuned to the
 * ultrasonic range. A common ultrasonic sensor, the Daventech SRF04
 * requires a short pulse to be generated on a digital channel. This
 * causes the chirp to be emmitted. A second line becomes high as the ping
 * is transmitted and goes low when the echo is received. The time that
 * the line is high determines the round trip distance (time of flight).
 */
class Ultrasonic : public SensorBase
{
  public:
    Ultrasonic(DigitalOutput * pingChannel, DigitalInput * echoChannel);
    Ultrasonic(DigitalOutput & pingChannel, DigitalInput & echoChannel);
    Ultrasonic(UINT32 pingChannel, UINT32 echoChannel);
    Ultrasonic(UINT32 pingSlot, UINT32 pingChannel,
               UINT32 echoSlot, UINT32 echoChannel);
    virtual ~Ultrasonic();

    void Ping();
    bool IsRangeValid();
    static void SetAutomaticMode(bool enabling);
    double GetRangeInches();
    double GetRangeMM();
    bool IsEnabled()
    {
        return m_enabled;
    }
    void SetEnabled(bool enable)
    {
        m_enabled = enable;
    }

  private:
    void Initialize();

    static void UltrasonicChecker();

    static const double kPingTime = 10 * 1e-6; ///< Time (sec) for the ping trigger pulse.
    static const UINT32 kPriority = 90; ///< Priority that the ultrasonic round robin task runs.
    static const double kMaxUltrasonicTime = 0.1; ///< Max time (ms) between readings.
    static const double kSpeedOfSoundInchesPerSec = 1130.0 * 12.0;

    static Task m_task;    // task doing the round-robin automatic sensing
    static Ultrasonic *m_firstSensor; // head of the ultrasonic sensor list
    static bool m_automaticEnabled;   // automatic round robin mode
    static SEM_ID m_semaphore; // synchronize access to the list of sensors

    DigitalInput *m_echoChannel;
    DigitalOutput *m_pingChannel;
    bool m_allocatedChannels;
    bool m_enabled;
    Counter *m_counter;
    Ultrasonic *m_nextSensor;
};

#endif
