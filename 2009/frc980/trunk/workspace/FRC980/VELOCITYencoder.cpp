#include "VELOCITYencoder.h"

double VELOCITYencoder::PIDGet()
{
	return GetRate();	//The encoder outputs the velocity
}