/********************************************************************************
*
*  Project   		: FIRST Motor Controller
*  Category 		: Team Deliverable Software (as source code) 	
*  Subsystem 		: FRC Vision for cRIO     			
* 
*  File Name  		: DashboardDemo.cpp        
*  Contributors 	: ELF, JDG
*  Creation Date 	: Dec 6, 2008
*  Revision History	: Source code and revision history are maintained at
*                               	   sourceforge.WPI.edu  
*  
*  File Description	: Demo program for sending images to the Dashboard
*   
*/                         
/*----------------------------------------------------------------------------*/
/*        Copyright (c) FIRST 2008.  All Rights Reserved.                     */
/*  Open Source Software - may be modified and shared by FRC teams. The code  */
/*  must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib. */
/*----------------------------------------------------------------------------*/

#include "vxWorks.h" 

#include "AxisCamera.h" 
#include "BaeUtilities.h"
#include "FrcError.h"
#include "PCVideoServer.h"

/** This example initializes the camera, then creates a PCVideoServer that starts
 * sending images to a dashboard at IP address 10.x.x.6.
 * Examples of stopping and restarting the PCVideo Server are included.
 */
void TestVision2PC(void) {
	char funcName[]="TestVision2PC";
	
	/* start camera task */
	if (StartCameraTask(13, 0, k320x240, ROT_0) == -1) {
		dprintf( LOG_ERROR,"Failed to spawn camera task; exiting. Error code %s", 
				GetVisionErrorText(GetLastVisionError()) );
	}
	dprintf(LOG_DEBUG,"Waiting for camera to init");
	Wait(2.0);

	// start up the task serving images to PC
	dprintf(LOG_DEBUG,"Starting ImageToPCServer");
	PCVideoServer pc;
	
	// this code demonstates stopping and restarting the PC image server
	int i;
	for (i = 0; i<60; i++) {
		Wait(1.0);
	}
	dprintf(LOG_DEBUG, "stopping image server");
	pc.Stop();   
	for (i = 0; i<10; i++) {
		Wait(1.0);
	}
	dprintf(LOG_DEBUG, "starting image server again");
	pc.Start();
	for (i = 0; i<999; i++) {
		Wait(1.0);
	}
	dprintf(LOG_DEBUG, "stopping image server");
	StopImageToPCServer();   //can use c or cpp call to stop
	dprintf(LOG_DEBUG, "TestVision2PC ending");
}


void RunProgram(void)
{
	char funcName[]="RunProgram";
	SetDebugFlag ( DEBUG_SCREEN_ONLY  ) ;
	dprintf (LOG_INFO, "Testing Image to Dashboard\n" ); 
	TestVision2PC();
}

/**
 * This is the main program that is run by the debugger or the robot on boot.
 **/
int DashboardDemo_StartupLibraryInit(void)
	{
		/*
		 * StartRobot((FUNCPTR) RunProgram);
		 */
		RunProgram();	// renamed
		return 0;
	}

