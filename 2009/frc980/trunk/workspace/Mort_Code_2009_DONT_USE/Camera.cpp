#include "MORT_defines.h"
#include "MORT_includes.h"

void ParPrintReport(ParticleAnalysisReport * myReport)
{
    cout << "\t cm_x_normalized = " << myReport->center_mass_x_normalized
        << " \t \t cm_y_normalized = " << myReport->
        center_mass_y_normalized << endl;
    cout << "\t particleArea = " << myReport->
        particleArea << "\t \t particleToImagePercent = " << myReport->
        particleToImagePercent << endl;
}

void ColPrintReport(ColorReport * myReport)
{
    cout << " --- ColorReport " << endl;

    cout << " \t particleHueMax = " << myReport->particleHueMax
        << " \t \t particleHueMin = " << myReport->particleHueMin << endl;

    cout << "\t particleHueMean = " << myReport->particleHueMean
        << "\t \t particleSatMax = " << myReport->particleSatMax << endl;

    cout << "\t particleSatMin = " << myReport->particleSatMin
        << "\t \t particleSatMean = " << myReport->particleSatMean << endl;

    cout << "\t particleLumMax = " << myReport->particleLumMax
        << "\t \t particleLumMin = " << myReport->particleLumMin << endl;

    cout << "\t particleLumMean = " << myReport->particleLumMean << endl;
}

void ThreshPrintReport(TrackingThreshold * myReport)
{
    cout << "\t hueMin = " << myReport->hue.minValue
        << "\t \t hueMax = " << myReport->hue.maxValue << endl;

    cout << "\t satMin = " << myReport->saturation.minValue
        << "\t \t satMax = " << myReport->saturation.maxValue << endl;

    cout << "\t lumMin = " << myReport->luminance.minValue
        << "\t \t lumMax = " << myReport->luminance.maxValue << endl;
}
