#include <WPILib.h>

#include "DashboardData.h"

DashboardData::DashboardData(void)
    : m_ds(DriverStation::GetInstance())
{

}

DashboardData::~DashboardData()
{

}

void DashboardData::PackAnalog(Dashboard *pDashPack, uint32_t u32Slot)
{
    AnalogModule *pAM = AnalogModule::GetInstance(u32Slot); // cRIO slot #

    pDashPack->AddCluster();
    for (uint32_t i = 1 ; i <= SensorBase::kAnalogChannels ; i++)
    {
        pDashPack->AddFloat(pAM->GetVoltage(i));
    }
    pDashPack->FinalizeCluster();
}

void DashboardData::PackDigital(Dashboard *pDashPack, uint32_t u32Slot)
{
    DigitalModule *pDM = DigitalModule::GetInstance(u32Slot);

    pDashPack->AddCluster();
    pDashPack->AddU8(0);        // m_RelayFwd[module]
    pDashPack->AddU8(0);        // m_RelayRev[module]

    uint16_t u16 = 0;

    for (int i = 1 ; i <= 14 ; i++)
    {
        u16 += GetDigitalInput(u32Slot, i) << (i-1);
    }

    pDashPack->AddU16(u16);     // m_DIOChannels[module]
    pDashPack->AddU16(0);       // m_DIOChannelsOutputEnable[module]

    pDashPack->AddCluster();
    for (uint32_t i = 1; i <= SensorBase::kPwmChannels; i++)
    {
        pDashPack->AddU8(pDM->GetPWM(i));
    }
    pDashPack->FinalizeCluster();

    pDashPack->FinalizeCluster();
}

/**
 * Pack data using the correct types and in the correct order to match the
 * default "Dashboard Datatype" in the LabVIEW Dashboard project.
 */
void DashboardData::UpdateAndSend(void)
{
    Dashboard & dashboardPacker = m_ds->GetDashboardPacker();

    // Pack the analog modules
    PackAnalog(&dashboardPacker, 1);
    PackAnalog(&dashboardPacker, 2);

    // Pack the digital modules
    PackDigital(&dashboardPacker, 4);
    PackDigital(&dashboardPacker, 6);

    // Pack the solenoid module
    dashboardPacker.AddU8(255); // m_SolenoidChannels

    // Flush the data to the driver station.
    dashboardPacker.Finalize();
}
