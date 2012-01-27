#include <WPILib.h>
#include <NetworkCommunication/FRCComm.h>
#include <Timer.h>
//#include <Vision/PCVideoServer.h>
#include <math.h>
#include <stdbool.h>

//#include "ReversableCANJaguar.h"
#include "CANJaguar.h"
#include "Robot980.h"
#include "numbers.h"
#include "utils.h"

#define ENC_SCALE   CounterBase::k1X /*!< \def ENC_SCALE The encoder scaling */

//==========================================================================
static Robot980 *g_pInstance = NULL;

//==========================================================================
//==========================================================================
Robot980::Robot980()
    //--- Jaguars (Encoders attached directly to Jaguars)
    // NOTE: CANNOT RUN IN kSpeed MODE UNLESS ENCODERS ATTACHED
    // left and right drive motors
    //: m_pscLeft1(new CANJaguar(CAN_LEFT_DRIVE1)) 
{
    //--- Encoder setup for Left CIMs
    //m_pscLeft1->ConfigEncoderCodesPerRev(US_DIGITAL_ENC_COUNTS);
    //m_pscLeft1->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
    //m_pscLeft1->ConfigMaxOutputVoltage(MAX_JAGUAR_OUTPUT_VOLTAGE);
    //m_pscLeft1->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);

    //--- Define Drive Timer
    m_pTimerDrive->Reset();
    m_pTimerDrive->Start();

    //--- Tell SensorBase about us
    AddToSingletonList();
}

//==========================================================================
Robot980::~Robot980()
{
    //delete m_pscLeft1;
    delete m_pTimerDrive;
}

//==========================================================================
Robot980 *Robot980::GetInstance()
{
    if (!g_pInstance)
    {
        g_pInstance = new Robot980();
    }
    return g_pInstance;
}

//==========================================================================
//==========================================================================
int Robot980::GetAutonMode()
{
    //--- Get the analog value corresponsind to the autonomous mode to choose
    AnalogModule *pAM = AnalogModule::GetInstance(SLOT_AUTO_MODE);
    int i = pAM->GetValue(CHAN_AUTO_MODE);      // returns 10-bit number

    if (i > 900)
        return 6;
    if (i > 700)
        return 5;
    if (i > 500)
        return 4;
    if (i > 300)
        return 3;
    if (i > 100)
        return 2;

    return 1;
}

//==========================================================================
void Robot980::SetBrakes(bool brakeOnStop)
{
    // kNeutralMode_Jumper, kNeutralMode_Brake, kNeutralMode_Coast
    CANJaguar::NeutralMode mode = brakeOnStop
        ? CANJaguar::kNeutralMode_Brake : CANJaguar::kNeutralMode_Coast;

    //m_pscLeft1->ConfigNeutralMode(mode);
}

//==========================================================================
void Robot980::Drive(float num)
{
    //--- Reset the Timer Drive
    m_pTimerDrive->Reset();
    //--- Set the speed of the left motors
    //m_pscLeft1->Set(utils::limit(num));
}

//==========================================================================
float Robot980::GetEncoder()
{
   return 0;
   // return m_pscLeft1->GetPosition() * 21.8;//3.14159 * WHEEL_DIAMETER;
}
