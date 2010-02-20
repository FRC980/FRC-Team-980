#include "WPILib.h"
#include "../CANJaguar/CANJaguar.h"

#include "sockLib.h"
#include "inetLib.h"
#include "hostLib.h"

/**
 * This is a demo program a CAN attached Jaguar running on-board closed-loop position control.
 * This is designed to interface with the Position Control dashboard for monitoring and tuning the controller.
 */ 
class CANRobotDemo : public SimpleRobot
{
	CANJaguar positionJag;	// CAN attached Jag for position control
	Joystick stick; // The setpoint joystick
	Joystick::AxisType axis; // The axis to use for the setpoint.
	Task commandTask; // Task for receiving commands from the dashboard
	static const UINT16 kDashboardCommandPort = 1200;

public:
	CANRobotDemo()
		: positionJag(4, CANJaguar::kPosition)
		, stick(1)
		, axis(Joystick::kXAxis)
		, commandTask("DashboardCommandServer", (FUNCPTR)DashboardCommandServerStub)
	{
		GetWatchdog().SetExpiration(100);
		positionJag.ConfigPotentiometerTurns(1);
		positionJag.ConfigEncoderCodesPerRev(360);
		positionJag.ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		positionJag.SetPositionReference(CANJaguar::kPosRef_Potentiometer);
		wpi_stackTraceEnable(true);
		commandTask.Start((INT32)this);
	}

	void UpdateDashboardStatus()
	{
		Dashboard &dashHigh = m_ds->GetHighPriorityDashboardPacker();
		dashHigh.AddCluster(); // PID (not used for now)
		dashHigh.AddDouble(0.0); // P
		dashHigh.AddDouble(0.0); // I
		dashHigh.AddDouble(0.0); // D
		dashHigh.FinalizeCluster();
		dashHigh.AddDouble(positionJag.GetPosition()); // Current position
		dashHigh.AddDouble(positionJag.Get()); // Setpoint
		dashHigh.AddDouble(positionJag.GetOutputVoltage()); // Output Voltage
		dashHigh.AddBoolean(positionJag.GetForwardLimitOK()); // Forward Limit
		dashHigh.AddBoolean(positionJag.GetReverseLimitOK()); // Reverse Limit
		dashHigh.Finalize();
	}

	/**
	 * Run the closed loop position controller on the Jag.
	 */
	void OperatorControl()
	{
		printf("In OperatorControl\n");
		GetWatchdog().SetEnabled(true);
		while (IsOperatorControl() && !IsDisabled())
		{
			GetWatchdog().Feed();
			// Set the desired setpoint
			positionJag.Set(stick.GetAxis(axis) * 0.5 + 0.5);
			UpdateDashboardStatus();
			Wait(0.05);
		}
	}
	void Disabled()
	{
		printf("In Disabled\n");
		while (IsDisabled())
		{
			UpdateDashboardStatus();
			Wait(0.05);
		}
	}
	static int DashboardCommandServerStub(CANRobotDemo *robot)
	{
		return robot->DashboardCommandServer();
	}
	// Shamelessly stolen from PCVideoServer
	int DashboardCommandServer()
	{
	    /* Setup to PC sockets */
	    struct sockaddr_in serverAddr;
	    int sockAddrSize = sizeof(serverAddr);
	    int pcSock = ERROR;
	    bzero ((char *) &serverAddr, sockAddrSize);
	    serverAddr.sin_len = (u_char) sockAddrSize;
	    serverAddr.sin_family = AF_INET;
	    serverAddr.sin_port = htons (kDashboardCommandPort);
	    serverAddr.sin_addr.s_addr = htonl (INADDR_ANY);

	    while (true)
	    {
	        taskSafe();
	    	  //  Create the socket.
	        if ((pcSock = socket (AF_INET, SOCK_STREAM, 0)) == ERROR)
	        {
	            perror ("socket");
	            continue;
	        }
	        //  Set the TCP socket so that it can be reused if it is in the wait state.
	        int reuseAddr = 1;
	        setsockopt(pcSock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char*>(&reuseAddr), sizeof(reuseAddr));
	        //  Bind socket to local address.
	        if (bind (pcSock, (struct sockaddr *) &serverAddr, sockAddrSize) == ERROR)
	        {
	            perror ("bind");
	            close (pcSock);
	            continue;
	        }
	        //  Create queue for client connection requests.
	        if (listen (pcSock, 1) == ERROR)
	        {
	            perror ("listen");
	            close (pcSock);
	            continue;
	        }

	        struct sockaddr_in clientAddr;
	        int clientAddrSize;
	        int newPCSock = accept (pcSock, reinterpret_cast<sockaddr*>(&clientAddr), &clientAddrSize);
	        if (newPCSock  == ERROR)
	        {
	            close(pcSock);
	            continue;
	        }

	        char cmdBuffer[32];
	        char *pBuffer;
	        while(1)
	        {
	            int numBytes = 0;
	            pBuffer = cmdBuffer;
	            while (numBytes < 2 || (*(pBuffer-2) != '\r' && *(pBuffer-1) != '\n'))
	            {
		            numBytes += read(newPCSock, pBuffer++, 1);
	            }
	            char command = cmdBuffer[0];
	            switch (command)
	            {
	            case 'E':
	            	positionJag.EnableControl();
	            	//printf("Enable\n");
	            	break;
	            case 'D':
	            	positionJag.DisableControl();
	            	//printf("Disable\n");
	            	break;
	            case 'G':
	            	{
	            		double P, I, D;
	            		memcpy((char*)&P, cmdBuffer+1, sizeof(double));
	            		memcpy((char*)&I, cmdBuffer+9, sizeof(double));
	            		memcpy((char*)&D, cmdBuffer+17, sizeof(double));
		            	positionJag.SetPID(P, I, D);
	            		//printf("Set- P: %f I: %f D: %f\n", P, I, D);
	            		//P = positionJag.GetP();
	            		//I = positionJag.GetI();
	            		//D = positionJag.GetD();
	            		//printf("Get- P: %f I: %f D: %f\n", P, I, D);
	            	}
	            	break;
	            case 'S':
	            	{
	            		double forward, reverse;
	            		memcpy((char*)&forward, cmdBuffer+1, sizeof(double));
	            		memcpy((char*)&reverse, cmdBuffer+9, sizeof(double));
	            		positionJag.ConfigSoftPositionLimits(forward, reverse);
	            		//printf("Soft Limits fwd: %f rev: %f\n", forward, reverse);
	            	}
	            	break;
	            case 'H':
	            	positionJag.DisableSoftPositionLimits();
	            	//printf("Limit switches only\n");
	            	break;
	            }

	            //no point in running too fast -
	            Wait(0.01);
	        }
	        //  Clean up
	        close (newPCSock);
	        newPCSock = ERROR;
	        close (pcSock);
	        pcSock = ERROR;
	        taskUnsafe();
	        Wait(0.1);
	    }
	    return (OK);
	}


};

START_ROBOT_CLASS(CANRobotDemo);

