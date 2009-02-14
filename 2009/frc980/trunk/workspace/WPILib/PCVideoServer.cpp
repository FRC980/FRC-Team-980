/*******************************************************************************
*  Project          : FIRST Motor Controller
*  File Name        : PCVideoServer.cpp
*  Contributors     : SVK, ELF
*  Creation Date    : November 30, 2008
*  Revision History : Source code & revision history maintained at
*                     sourceforge.WPI.edu
*  File Description : C++ Axis camera control for the FIRST Vision API
*/
/*----------------------------------------------------------------------------*/
/*        Copyright (c) FIRST 2008.  All Rights Reserved.                     */
/*  Open Source Software - may be modified and shared by FRC teams. The code  */
/*  must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib. */
/*----------------------------------------------------------------------------*/

#include <vxWorks.h>

#include "sockLib.h"
#include "inetLib.h"
#include "hostLib.h"

#include "AxisCamera.h"
#include "BaeUtilities.h"
#include "chipobject/NiRioStatus.h"
#include "Task.h"
#include "Timer.h"
#include "Utility.h"

#include "PCVideoServer.h"

bool g_stopImageToPCServer = false;

/**
@brief Implements an object that automatically does a close on a
camera socket on destruction.
*/
class ScopedSocket
{
  public:
    ScopedSocket(int camSock,
                 const ErrorBase * errorBase):m_camSock(camSock)
    {
        if (errorBase->StatusIsFatal())
        {
            return;
        }
        if   (m_camSock == ERROR)
        {
            wpi_setError(*errorBase, ERR_CAMERA_SOCKET_CREATE_FAILED);
            return;
        }
    }

    ~ScopedSocket()
    {
        if (m_camSock != ERROR)
        {
            close(m_camSock);
        }
    }
    //  Cast to int allows you to pass this to any function that
    //  takes the socket as an int.
    operator    int () const
    {
        return m_camSock;
    }

  private:
    int  m_camSock;
};

//============================================================================
//  PCVideoServer
//============================================================================

/**
@brief Constructor.
*/
PCVideoServer::PCVideoServer(void):m_task("PCVideo",
                                          (FUNCPTR) ImageToPCServer)
{
    try
    {
        printf("START PCVideoServer constructor\n");
        // Start the image communication task.
        StartPcTask();
        if (StatusIsFatal())
        {
            throw GetError().GetCode();
        }
    }
    catch(Error::Code error)
    {
        wpi_setError(*this, error);
        return;
    }
}

/**
@brief Destructor.
Stop serving images and destroy this class.
*/
PCVideoServer::~PCVideoServer()
{
    printf("START PCVideoServer DESTRUCTOR **********************\n");
    //  Stop the images to PC server.
    Stop();
    //StopImageToPCServer();
    //  Clear the error so that you can use this object to make a connection to
    //  the VIDEO_TO_PC_PORT to stop the ImageToPCServer if it is waiting to
    //  accept connections from a PC.
    ClearError();
    //  Open a socket.
    ScopedSocket camSock(socket(AF_INET, SOCK_STREAM, 0), this);
    //  If successful
    if (!StatusIsFatal())
    {
        //  Create a connection to the localhost.
        struct sockaddr_in selfAddr;
        int  sockAddrSize = sizeof(selfAddr);
        bzero((char *)&selfAddr, sockAddrSize);
        selfAddr.sin_family = AF_INET;
        selfAddr.sin_len = (u_char) sockAddrSize;
        selfAddr.sin_port = htons(VIDEO_TO_PC_PORT);

        if (((int)
             (selfAddr.sin_addr.s_addr =
              inet_addr(const_cast < char *>("localhost"))) != ERROR)
            ||
            ((int)
             (selfAddr.sin_addr.s_addr =
              hostGetByName(const_cast < char *>("localhost"))) != ERROR))
        {
            connect(camSock, (struct sockaddr *)&selfAddr, sockAddrSize);
        }
    }
}

/**
 * Start the task that is responsible for sending images to the PC.
 */
int PCVideoServer::StartPcTask()
{
    if (StatusIsFatal())
    {
        return -1;
    }
    int id = 0;
    g_stopImageToPCServer = false;
    // check for prior copy of running task
    // TODO: Report error. You are in a bad state.
    //char taskName[1024];
    //sprintf(taskName, "%s%X", taskName, reinterpret_cast<int>(this));
    int oldId = taskNameToId(m_task.GetName());
    if (oldId != ERROR)
    {
        taskDelete(oldId);
    }

    // spawn video server task
    // this is done to ensure that the task is spawned with the
    // floating point context save parameter.
    bool started = m_task.Start();

    id = m_task.GetID();

    if (!started)
    {
        wpi_setError(*this, ERR_CAMERA_TASK_SPAWN_FAILED);
        return id;
    }
    taskDelay(1);
    return id;
}

/**
@brief Start sending images to the PC.
*/
void PCVideoServer::Start()
{
    StartPcTask();
}

/**
@brief Stop sending images to the PC.
*/
void PCVideoServer::Stop()
{
    printf("Stop");
    g_stopImageToPCServer = true;
}

//============================================================================
//  C Functions
//============================================================================

/**
@brief Stop sending images to the PC.
*/
void StopImageToPCServer()
{
    printf("StopImageToPCServer");
    g_stopImageToPCServer = true;
}

/**
@brief Check is server is shutting down.
@return bool true if time to stop
*/
bool ShouldStopImageToPCServer()
{
    //  Return the current state of g_stopImageToPCServer
    return g_stopImageToPCServer;
}

/**
@brief Initialize the socket and serve images to the PC.
*/
int ImageToPCServer()
{
    printf("ImageToPCServer Started\n");

    /* Setup to PC sockets */
    struct sockaddr_in serverAddr;
    int sockAddrSize = sizeof(serverAddr);
    int pcSock = ERROR;
    bzero((char *)&serverAddr, sockAddrSize);
    serverAddr.sin_len = (u_char) sockAddrSize;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(VIDEO_TO_PC_PORT);
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    int success;
    while (!ShouldStopImageToPCServer())
    {
        double lastImageTimestamp = 0;
        //  Create the socket.
        if ((pcSock = socket(AF_INET, SOCK_STREAM, 0)) == ERROR)
        {
            perror("socket");
            continue;
        }
        //  Set the TCP socket so that it can be reused if it is in the
        //  wait state.
        int reuseAddr = 1;
        setsockopt(pcSock, SOL_SOCKET, SO_REUSEADDR,
                   reinterpret_cast < char *>(&reuseAddr),
                   sizeof(reuseAddr));
        //  Bind socket to local address.
        // printf ("Binding to socket\n");
        if (bind(pcSock, (struct sockaddr *)&serverAddr, sockAddrSize)
            == ERROR)
        {
            perror("bind");
            close(pcSock);
            continue;
        }
        //  Create queue for client connection requests.
        printf("Listening on socket (port:%d)\n", VIDEO_TO_PC_PORT);
        if (listen(pcSock, 1) == ERROR)
        {
            perror("listen");
            close(pcSock);
            continue;
        }
        //  Accept a new connect request
        struct sockaddr_in clientAddr;
        int clientAddrSize;
        printf("accept socket\n");
        int newPCSock =
            accept(pcSock, reinterpret_cast < sockaddr * >(&clientAddr),
                   &clientAddrSize);
        if (newPCSock == ERROR)
        {
            perror("accept");
            close(pcSock);
            continue;
        }
        printf("CONNECTED TO PC socket:%d\n", newPCSock);
        //  Got a connection.
        //TODO: check camera error
        //if (!StatusIsFatal()) {
        if (1)
        {
            printf
                ("Everyting is OKAY...starting to get images from camera\n");
            while (!ShouldStopImageToPCServer())
            {
                double newTimestamp;
                int numBytes = 0;
                char *imageData = NULL;
                //printf ("Getting image from camera\n");
                success = GetImageDataBlocking(&imageData,
                                               &numBytes,
                                               &newTimestamp,
                                               lastImageTimestamp);
                if (!success)
                {
                    //  If camera is not initialzed you will get failure
                    //  and the timestamp is invalid. Reset this value and
                    //  try again.
                    lastImageTimestamp = 0;
                    continue;
                }
                lastImageTimestamp = newTimestamp;

                //printf ("Writing image to PC\n");
                /* Write header to PC */
                static const char header[4] = { 1, 0, 0, 0 };
                int headerSend = write(newPCSock,
                                       const_cast < char *>(header), 4);

                /* Write image length to PC */
                int presend = write(newPCSock,
                                    reinterpret_cast < char *>(&numBytes),
                                    4);

                /* Write image to PC */
                int sent = write(newPCSock, imageData, numBytes);

                //  Cleanup memory allocated for the image.
                delete imageData;
                imageData = NULL;
                numBytes = 0;

                //  The PC probably closed connection. Get out of here
                //  and try listening again.
                if (headerSend == ERROR || sent == ERROR
                    || presend == ERROR)
                {
                    break;
                }
                //no point in running too fast -
                //max camera frame rate is 30 fps
                Wait(0.01);
            }
        }
        printf("CLEANING UP...\n");
        //  Clean up
        close(newPCSock);
        newPCSock = ERROR;
        close(pcSock);
        pcSock = ERROR;
    }
    return (OK);
}
