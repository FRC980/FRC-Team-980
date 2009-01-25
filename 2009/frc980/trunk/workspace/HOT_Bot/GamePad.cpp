#include "GamePad.h"
#include "DriverStation.h"
#include "Utility.h"
#include "WPIStatus.h"

/**
 * Construct an instance of a gamepad.
 * The gamepad index is the usb port on the drivers station.
 * 
 * @param port The port on the driver station that the joystick is plugged into.
 */
GamePad::GamePad(unsigned port):m_ds(NULL), m_port(port), m_axes(NULL),
m_buttons(NULL)
{
    InitGamePad(kNumAxisTypes, kNumButtonTypes);

    m_axes[kLeftXAxis] = kDefaultLeftXAxis;
    m_axes[kLeftYAxis] = kDefaultLeftYAxis;
    m_axes[kRightXAxis] = kDefaultRightXAxis;
    m_axes[kRightYAxis] = kDefaultRightYAxis;
    m_axes[kRockerXAxis] = kDefaultRockerXAxis;
    m_axes[kRockerYAxis] = kDefaultRockerYAxis;

    m_buttons[kButton01] = kDefaultButton01;
    m_buttons[kButton02] = kDefaultButton02;
    m_buttons[kButton03] = kDefaultButton03;
    m_buttons[kButton04] = kDefaultButton04;
    m_buttons[kButton05] = kDefaultButton05;
    m_buttons[kButton06] = kDefaultButton06;
    m_buttons[kButton07] = kDefaultButton07;
    m_buttons[kButton08] = kDefaultButton08;
    m_buttons[kButton09] = kDefaultButton09;
    m_buttons[kButton10] = kDefaultButton10;
    m_buttons[kButton11] = kDefaultButton11;
    m_buttons[kButton12] = kDefaultButton12;
}

void GamePad::InitGamePad(unsigned numAxisTypes, unsigned numButtonTypes)
{
    m_ds = DriverStation::GetInstance();
    m_axes = new unsigned[numAxisTypes];
    m_buttons = new unsigned[numButtonTypes];
}

GamePad::~GamePad()
{
    delete[]m_buttons;
    delete[]m_axes;
}

/**
 * Get the X value of the gamepad's left joystick.
 */
float GamePad::GetLeftX(void)
{
    return GetRawAxis(m_axes[kLeftXAxis]);
}

/**
 * Get the Y value of the gamepad's left joystick.
 */
float GamePad::GetLeftY(void)
{
    return -GetRawAxis(m_axes[kLeftYAxis]);
}

/**
 * Get the X value of the gamepad's right joystick.
 */
float GamePad::GetRightX(void)
{
    return GetRawAxis(m_axes[kRightXAxis]);
}

/**
 * Get the Y value of the gamepad's right joystick.
 */
float GamePad::GetRightY(void)
{
    return -GetRawAxis(m_axes[kRightYAxis]);
}

/**
 * Get the X value of the gamepad's rocker.
 */
float GamePad::GetRockerX(void)
{
    return GetRawAxis(m_axes[kRockerXAxis]);
}

/**
 * Get the Y value of the gamepad's rocker.
 */
float GamePad::GetRockerY(void)
{
    return GetRawAxis(m_axes[kRockerYAxis]);
}

/**
 * Get the value of the axis.
 * 
 * @param axis The axis to read [1-6].
 * @return The value of the axis.
 */
float GamePad::GetRawAxis(unsigned axis)
{
    return m_ds->GetStickAxis(m_port, axis);
}

/**
 * For the current gamepad, return the axis determined by the argument.
 * 
 * This is for cases where the gamepad axis is returned programatically, otherwise one of the
 * previous functions would be preferable (for example GetLeftX()).
 * 
 * @param axis The axis to read.
 * @return The value of the axis.
 */
float GamePad::GetAxis(AxisType axis)
{
    switch (axis)
    {
    case kLeftXAxis:
        return this->GetLeftX();
    case kLeftYAxis:
        return -this->GetLeftY();
    case kRightXAxis:
        return this->GetRightX();
    case kRightYAxis:
        return -this->GetRightY();
    case kRockerXAxis:
        return this->GetRockerX();
    case kRockerYAxis:
        return this->GetRockerY();
    default:
        wpi_fatal(BadJoystickAxis);
        return 0.0;
    }
}

/**
 * Read the state of button 1 on the gamepad.
 * 
 * @return The state of the button.
 */
bool GamePad::GetButton01(void)
{
    return GetRawButton(m_buttons[kButton01]);
}

/**
 * Read the state of button 2 on the gamepad.
 * 
 * @return The state of the button.
 */
bool GamePad::GetButton02(void)
{
    return GetRawButton(m_buttons[kButton02]);
}

/**
 * Read the state of button 3 on the gamepad.
 * 
 * @return The state of the button.
 */
bool GamePad::GetButton03(void)
{
    return GetRawButton(m_buttons[kButton03]);
}

/**
 * Read the state of button 4 on the gamepad.
 * 
 * @return The state of the button.
 */
bool GamePad::GetButton04(void)
{
    return GetRawButton(m_buttons[kButton04]);
}

/**
 * Read the state of button 5 on the gamepad.
 * 
 * @return The state of the button.
 */
bool GamePad::GetButton05(void)
{
    return GetRawButton(m_buttons[kButton05]);
}

/**
 * Read the state of button 6 on the gamepad.
 * 
 * @return The state of the button.
 */
bool GamePad::GetButton06(void)
{
    return GetRawButton(m_buttons[kButton06]);
}

/**
 * Read the state of button 7 on the gamepad.
 * 
 * @return The state of the button.
 */
bool GamePad::GetButton07(void)
{
    return GetRawButton(m_buttons[kButton07]);
}

/**
 * Read the state of button 8 on the gamepad.
 * 
 * @return The state of the button.
 */
bool GamePad::GetButton08(void)
{
    return GetRawButton(m_buttons[kButton08]);
}

/**
 * Read the state of button 9 on the gamepad.
 * 
 * @return The state of the button.
 */
bool GamePad::GetButton09(void)
{
    return GetRawButton(m_buttons[kButton09]);
}

/**
 * Read the state of button 10 on the gamepad.
 * 
 * @return The state of the button.
 */
bool GamePad::GetButton10(void)
{
    return GetRawButton(m_buttons[kButton10]);
}

/**
 * Read the state of button 11 on the gamepad.
 * 
 * @return The state of the button.
 */
bool GamePad::GetButton11(void)
{
    return GetRawButton(m_buttons[kButton11]);
}

/**
 * Read the state of button 12 on the gamepad.
 * 
 * @return The state of the button.
 */
bool GamePad::GetButton12(void)
{
    return GetRawButton(m_buttons[kButton12]);
}

/**
 * Get the button value for buttons 1 through 12.
 * 
 * The buttons are returned in a single 16 bit value with one bit representing the state
 * of each button. The appropriate button is returned as a boolean value. 
 * 
 * @param button The button number to be read.
 * @return The state of the button.
 **/
bool GamePad::GetRawButton(unsigned button)
{
    return ((0x1 << (button - 1)) & m_ds->GetStickButtons(m_port)) != 0;
}

/**
 * Get buttons based on an enumerated type.
 * 
 * The button type will be looked up in the list of buttons and then read.
 * 
 * @param button The type of button to read.
 * @return The state of the button.
 */
bool GamePad::GetButton(ButtonType button)
{
    switch (button)
    {
    case kButton01:
        return GetButton01();
    case kButton02:
        return GetButton02();
    case kButton03:
        return GetButton03();
    case kButton04:
        return GetButton04();
    case kButton05:
        return GetButton05();
    case kButton06:
        return GetButton06();
    case kButton07:
        return GetButton07();
    case kButton08:
        return GetButton08();
    case kButton09:
        return GetButton09();
    case kButton10:
        return GetButton10();
    case kButton11:
        return GetButton11();
    case kButton12:
        return GetButton12();
    default:
        wpi_assert(false);
        return false;
    }
}
