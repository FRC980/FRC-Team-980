/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "Dashboard.h"
#include "Synchronized.h"
#include "Utility.h"
#include "WPIStatus.h"
#include <strLib.h>

/**
 * Dashboard contructor.
 * 
 * This is only called once when the DriverStation constructor is called.
 */
Dashboard::Dashboard(char **userStatus)
	: m_userStatus (userStatus)
	, m_localBuffer (NULL)
	, m_localPrintBuffer (NULL)
	, m_packPtr (NULL)
	, m_printSemaphore (0)
	, m_sequence (0)
{
	m_localBuffer = new char[kMaxDashboardDataSize];
	m_localPrintBuffer = new char[kMaxDashboardDataSize * 2];
	m_localPrintBuffer[0] = 0;
	m_packPtr = m_localBuffer;
	m_printSemaphore = semMCreate(SEM_DELETE_SAFE | SEM_INVERSION_SAFE); // synchronize access to multi-value registers
}

/**
 * Dashboard destructor.
 * 
 * Called only when the DriverStation class is destroyed.
 */
Dashboard::~Dashboard()
{
	m_packPtr = NULL;
	m_userStatus = NULL;
	delete [] m_localPrintBuffer;
	m_localPrintBuffer = NULL;
	delete [] m_localBuffer;
	m_localBuffer = NULL;
}

/**
 * Pack a signed 8-bit int into the dashboard data structure.
 * @param value Data to be packed into the structure.
 */
void Dashboard::AddI8(INT8 value)
{
	if (!ValidateAdd(sizeof(INT8))) return;
	memcpy(m_packPtr, (char*)&value, sizeof(value));
	m_packPtr += sizeof(value);
	AddedElement(kI8);
}

/**
 * Pack a signed 16-bit int into the dashboard data structure.
 * @param value Data to be packed into the structure.
 */
void Dashboard::AddI16(INT16 value)
{
	if (!ValidateAdd(sizeof(INT16))) return;
	memcpy(m_packPtr, (char*)&value, sizeof(value));
	m_packPtr += sizeof(value);
	AddedElement(kI16);
}

/**
 * Pack a signed 32-bit int into the dashboard data structure.
 * @param value Data to be packed into the structure.
 */
void Dashboard::AddI32(INT32 value)
{
	if (!ValidateAdd(sizeof(INT32))) return;
	memcpy(m_packPtr, (char*)&value, sizeof(value));
	m_packPtr += sizeof(value);
	AddedElement(kI32);
}

/**
 * Pack an unsigned 8-bit int into the dashboard data structure.
 * @param value Data to be packed into the structure.
 */
void Dashboard::AddU8(UINT8 value)
{
	if (!ValidateAdd(sizeof(UINT8))) return;
	memcpy(m_packPtr, (char*)&value, sizeof(value));
	m_packPtr += sizeof(value);
	AddedElement(kU8);
}

/**
 * Pack an unsigned 16-bit int into the dashboard data structure.
 * @param value Data to be packed into the structure.
 */
void Dashboard::AddU16(UINT16 value)
{
	if (!ValidateAdd(sizeof(UINT16))) return;
	memcpy(m_packPtr, (char*)&value, sizeof(value));
	m_packPtr += sizeof(value);
	AddedElement(kU16);
}

/**
 * Pack an unsigned 32-bit int into the dashboard data structure.
 * @param value Data to be packed into the structure.
 */
void Dashboard::AddU32(UINT32 value)
{
	if (!ValidateAdd(sizeof(UINT32))) return;
	memcpy(m_packPtr, (char*)&value, sizeof(value));
	m_packPtr += sizeof(value);
	AddedElement(kU32);
}

/**
 * Pack a 32-bit floating point number into the dashboard data structure.
 * @param value Data to be packed into the structure.
 */
void Dashboard::AddFloat(float value)
{
	if (!ValidateAdd(sizeof(float))) return;
	memcpy(m_packPtr, (char*)&value, sizeof(value));
	m_packPtr += sizeof(value);
	AddedElement(kFloat);
}

/**
 * Pack a 64-bit floating point number into the dashboard data structure.
 * @param value Data to be packed into the structure.
 */
void Dashboard::AddDouble(double value)
{
	if (!ValidateAdd(sizeof(double))) return;
	memcpy(m_packPtr, (char*)&value, sizeof(value));
	m_packPtr += sizeof(value);
	AddedElement(kDouble);
}

/**
 * Pack a boolean into the dashboard data structure.
 * @param value Data to be packed into the structure.
 */
void Dashboard::AddBoolean(bool value)
{
	if (!ValidateAdd(sizeof(char))) return;
	*m_packPtr = value ? 1 : 0;
	m_packPtr += sizeof(char);	
	AddedElement(kBoolean);
}

/**
 * Pack a NULL-terminated string of 8-bit characters into the dashboard data structure.
 * @param value Data to be packed into the structure.
 */
void Dashboard::AddString(char* value)
{
	AddString(value, strlen(value));
}

/**
 * Pack a string of 8-bit characters of specified length into the dashboard data structure.
 * @param value Data to be packed into the structure.
 * @param length The number of bytes in the string to pack.
 */
void Dashboard::AddString(char* value, INT32 length)
{
	if (!ValidateAdd(length + sizeof(length))) return;
	memcpy(m_packPtr, (char*)&length, sizeof(length));
	m_packPtr += sizeof(length);
	memcpy(m_packPtr, (char*)&value, length);
	m_packPtr += length;
	AddedElement(kString);
}

/**
 * Start an array in the packed dashboard data structure.
 * 
 * After calling AddArray(), call the appropriate Add method for each element of the array.
 * Make sure you call the same add each time.  An array must contain elements of the same type.
 * You can use clusters inside of arrays to make each element of the array contain a structure of values.
 * You can also nest arrays inside of other arrays.
 * Every call to AddArray() must have a matching call to FinalizeArray().
 */
void Dashboard::AddArray(void)
{
	if (!ValidateAdd(sizeof(INT32))) return;
	m_complexTypeStack.push(kArray);
	m_arrayElementCount.push_back(0);
	m_arraySizePtr.push_back((INT32*)m_packPtr);
	m_packPtr += sizeof(INT32);
}

/**
 * Indicate the end of an array packed into the dashboard data structure.
 * 
 * After packing data into the array, call FinalizeArray().
 * Every call to AddArray() must have a matching call to FinalizeArray().
 */
void Dashboard::FinalizeArray(void)
{
	if (m_complexTypeStack.top() != kArray)
	{
		wpi_fatal(MismatchedComplexTypeClose);
		return;
	}
	m_complexTypeStack.pop();
	*(m_arraySizePtr.back()) = m_arrayElementCount.back();
	m_arraySizePtr.pop_back();
	if (m_arrayElementCount.back() != 0)
	{
		m_expectedArrayElementType.pop_back();
	}
	m_arrayElementCount.pop_back();
	AddedElement(kOther);
}

/**
 * Start a cluster in the packed dashboard data structure.
 * 
 * After calling AddCluster(), call the appropriate Add method for each element of the cluster.
 * You can use clusters inside of arrays to make each element of the array contain a structure of values.
 * Every call to AddCluster() must have a matching call to FinalizeCluster().
 */
void Dashboard::AddCluster(void)
{
	m_complexTypeStack.push(kCluster);
}

/**
 * Indicate the end of a cluster packed into the dashboard data structure.
 * 
 * After packing data into the cluster, call FinalizeCluster().
 * Every call to AddCluster() must have a matching call to FinalizeCluster().
 */
void Dashboard::FinalizeCluster(void)
{
	if (m_complexTypeStack.top() != kCluster)
	{
		wpi_fatal(MismatchedComplexTypeClose);
		return;
	}
	m_complexTypeStack.pop();
	AddedElement(kOther);
}

/**
 * Print a string to the UserData text on the Dashboard.
 * 
 * This will add text to the buffer to send to the dashboard.
 * You must call Finalize() periodically to actually send the buffer to the dashboard if you are not using the packed dashboard data.
 */
void Dashboard::Printf(const char *writeFmt, ...)
{
	va_list args;
	INT32 size;

	va_start (args, writeFmt);
	{
		Synchronized sync(m_printSemaphore);
		vsprintf(m_localPrintBuffer + strlen(m_localPrintBuffer), writeFmt, args);
		size = strlen(m_localPrintBuffer);
	}
	if (size > kMaxDashboardDataSize)
	{
		wpi_fatal(DashboardDataOverflow);
	}

	va_end (args);
}

/**
 * Indicate that the packing is complete and commit the buffer to the DriverStation.
 * 
 * The packing of the dashboard packet is complete.
 * If you are not using the packed dashboard data, you can call Finalize() to commit the Printf() buffer and the error string buffer.
 * In effect, you are packing an empty structure.
 * Prepares a packet to go to the dashboard...
 * Pack the sequence number, Printf() buffer, the errors messages (not implemented yet), and packed dashboard data buffer.
 * @return The total size of the data packed into the userData field of the status packet.
 */
INT32 Dashboard::Finalize(void)
{
	if (*m_userStatus == NULL)
	{
		wpi_fatal(NullParameter);
		return 0;
	}
	if (!m_complexTypeStack.empty())
	{
		wpi_fatal(MismatchedComplexTypeClose);
		return 0;
	}

	INT32 size = 0;

	// Sequence number
	memcpy(*m_userStatus + size, &m_sequence, sizeof(m_sequence));
	size += sizeof(m_sequence);
	m_sequence++;

	// User printed strings
	INT32 printSize;
	{
		Synchronized sync(m_printSemaphore);
		printSize = strlen(m_localPrintBuffer);
		memcpy(*m_userStatus + size, &printSize, sizeof(printSize));
		size += sizeof(printSize);
		memcpy(*m_userStatus + size, m_localPrintBuffer, printSize);
		size += printSize;
		m_localPrintBuffer[0] = 0;
	}

	// Error Strings
	INT32 errorSize = 0;
	if (printSize + errorSize > kMaxDashboardDataSize)
	{
		wpi_fatal(DashboardDataOverflow);
		return 0;
	}
	memcpy(*m_userStatus + size, &errorSize, sizeof(errorSize));
	size += sizeof(errorSize);
	///< TODO: add error reporting strings.
	size += errorSize;

	// Dashboard Data
	INT32 dataSize = m_packPtr - m_localBuffer;
	if (printSize + errorSize + dataSize > kMaxDashboardDataSize)
	{
		wpi_fatal(DashboardDataOverflow);
		return 0;
	}
	memcpy(*m_userStatus + size, &dataSize, sizeof(dataSize));
	size += sizeof(dataSize);
	memcpy(*m_userStatus + size, m_localBuffer, dataSize);
	size += dataSize;
	m_packPtr = m_localBuffer;

	return size;
}

/**
 * Validate that the data being packed will fit in the buffer.
 */
bool Dashboard::ValidateAdd(INT32 size)
{
	if ((m_packPtr - m_localBuffer) + size > kMaxDashboardDataSize)
	{
		wpi_fatal(DashboardDataOverflow);
		return false;
	}
	return true;
}

/**
 * Check for consistent types when adding elements to an array and keep track of the number of elements in the array.
 */
void Dashboard::AddedElement(Type type)
{
	if(IsArrayRoot())
	{
		if (m_arrayElementCount.back() == 0)
		{
			m_expectedArrayElementType.push_back(type);
		}
		else
		{
			if (type != m_expectedArrayElementType.back())
			{
				wpi_fatal(InconsistentArrayValueAdded);
			}
		}
		m_arrayElementCount.back() = m_arrayElementCount.back() + 1;
	}
}

/**
 * If the top of the type stack an array?
 */
bool Dashboard::IsArrayRoot(void)
{
	return !m_complexTypeStack.empty() && m_complexTypeStack.top() == kArray;
}

