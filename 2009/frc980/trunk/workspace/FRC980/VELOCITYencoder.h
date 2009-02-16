#ifndef VELOCITYencoder_H
#define VELOCITYencoder_H

#include "PIDSource.h"
#include "Encoder.h"

class VELOCITYencoder: public PIDSource, public Encoder 
{
	public:
	virtual double PIDGet();  //Overides the PIDSource PIDGet()
}

#endif
