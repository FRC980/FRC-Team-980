#ifndef GAME_PAD_H_
#define GAME_PAD_H_

#include "GenericHID.h"
#include "Base.h"

class DriverStation;

/**
 * Handle input from standard Gamepads connected to the Driver Station.
 * This class handles standard input that comes from the Driver Station. Each time a value is requested
 * the most recent value is returned. There is a single class instance for each gamepad and the mapping
 * of ports to hardware buttons depends on the code in the driver station.
 */
class GamePad
{
  public:
    static const unsigned kDefaultLeftXAxis = 1;
    static const unsigned kDefaultLeftYAxis = 2;
    static const unsigned kDefaultRightXAxis = 3;
    static const unsigned kDefaultRightYAxis = 4;
    static const unsigned kDefaultRockerXAxis = 5;
    static const unsigned kDefaultRockerYAxis = 6;
    typedef enum
    {
        kLeftXAxis, kLeftYAxis, kRightXAxis, kRightYAxis, kRockerXAxis,
        kRockerYAxis, kNumAxisTypes
    } AxisType;
    static const unsigned kDefaultButton01 = 1;
    static const unsigned kDefaultButton02 = 2;
    static const unsigned kDefaultButton03 = 3;
    static const unsigned kDefaultButton04 = 4;
    static const unsigned kDefaultButton05 = 5;
    static const unsigned kDefaultButton06 = 6;
    static const unsigned kDefaultButton07 = 7;
    static const unsigned kDefaultButton08 = 8;
    static const unsigned kDefaultButton09 = 9;
    static const unsigned kDefaultButton10 = 10;
    static const unsigned kDefaultButton11 = 11;
    static const unsigned kDefaultButton12 = 12;
    typedef enum
    {
        kButton01, kButton02, kButton03, kButton04, kButton05, kButton06,
            kButton07,
        kButton08, kButton09, kButton10, kButton11, kButton12,
            kNumButtonTypes
    } ButtonType;

         GamePad(unsigned port);
         virtual ~ GamePad();

    unsigned GetAxisChannel(AxisType axis);
    void SetAxisChannel(AxisType axis, unsigned channel);

    virtual float GetLeftX(void);
    virtual float GetLeftY(void);
    virtual float GetRightX(void);
    virtual float GetRightY(void);
    virtual float GetRockerX(void);
    virtual float GetRockerY(void);
    virtual float GetAxis(AxisType axis);
    float GetRawAxis(unsigned axis);

    virtual bool GetButton01(void);
    virtual bool GetButton02(void);
    virtual bool GetButton03(void);
    virtual bool GetButton04(void);
    virtual bool GetButton05(void);
    virtual bool GetButton06(void);
    virtual bool GetButton07(void);
    virtual bool GetButton08(void);
    virtual bool GetButton09(void);
    virtual bool GetButton10(void);
    virtual bool GetButton11(void);
    virtual bool GetButton12(void);
    virtual bool GetButton(ButtonType button);
    bool GetRawButton(unsigned button);

  private:
    void InitGamePad(unsigned numAxisTypes, unsigned numButtonTypes);

    DriverStation *m_ds;
    unsigned m_port;
    unsigned *m_axes;
    unsigned *m_buttons;
};

#endif
