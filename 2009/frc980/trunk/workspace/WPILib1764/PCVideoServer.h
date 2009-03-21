/********************************************************************************
*  Project   		: FIRST Motor Controller
*  File Name  		: PCVideoServer.h
*  Contributors   	: ELF
*  Creation Date 	: November 29, 2008
*  Revision History	: Source code & revision history maintained at sourceforge.WPI.edu
*  File Description	: C++ Axis camera control for the FIRST Vision API
*/
/*----------------------------------------------------------------------------*/
/*        Copyright (c) FIRST 2008.  All Rights Reserved.                     */
/*  Open Source Software - may be modified and shared by FRC teams. The code  */
/*  must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib. */
/*----------------------------------------------------------------------------*/

#ifndef __PCVIDEOSERVER_H__
#define __PCVIDEOSERVER_H__

#include "Task.h"

/* C methods that may be called directly or from C++ class */
int ImageToPCServer();
bool ShouldStopImageToPCServer();
void StopImageToPCServer();

/**
 * Class the serves images to the PC.
 */
class PCVideoServer : public ErrorBase {

public:
	PCVideoServer(void);
    ~PCVideoServer();
    unsigned int Release();
    void Start();
    void Stop();

private:
    int StartPcTask();
    Task m_task;
};

#endif


