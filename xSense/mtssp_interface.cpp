
//  Copyright (c) 2003-2020 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "my_assert.h"
#include "mtssp_interface.h"
#include "xbusdef.h"
#include "common.h"

/*!	\class MtsspInterface
	\brief Interface for communicating with an Xsens Motion Tracker via MTSSP (MTi Synchronous Serial Protocol)
*/


/*!	\brief Constructs an MtsspInterface
	\param[in] driver The MtsspDriver for handling the communication with the device
*/
MtsspInterface::MtsspInterface(MtsspDriver* driver)
	: m_driver(driver)
{
}


/*!	\brief Read MTSSP protocol info
	\param[out] version: The version byte
	\param[out] dataReadyConfig: The data ready configuration byte
	\sa configureProtocol
*/
void MtsspInterface::readProtocolInfo(uint8_t& version, uint8_t& dataReadyConfig)
{
	uint8_t buffer[2];
	m_driver->read(XBUS_PROTOCOL_INFO, buffer, 2);
	version = buffer[0];
	dataReadyConfig = buffer[1];
}


/*!	\brief Write MTSSP protocol settings
	\param[in] dataReadyConfig The data ready configuration which must be set

	Bit 7:4	Reserved \n
	Bit 3	Measurement pipe DRDY event enable: 0 = disabled, 1 = enabled \n
	Bit 2	Notification pipe DRDY event enable: 0 = disabled, 1 = enabled \n
	Bit 1	Output type of DRDY pin: = 0 Push/pull, 1 = open drain \n
	Bit 0	Polarity of DRDY signal: 0 = Idle low, 1 = Idle high \n
	\sa readProtocolInfo
*/
void MtsspInterface::configureProtocol(uint8_t dataReadyConfig)
{
	m_driver->write(XBUS_CONFIGURE_PROTOCOL, &dataReadyConfig, sizeof(dataReadyConfig));
}

static COMMON uint8_t status[4];

/*!	\brief Read the pipe status
	\param[out] notificationMessageSize: The number of pending notification bytes
	\param[out] measurementMessageSize: The number of pending measurement bytes
*/
void MtsspInterface::readPipeStatus(uint16_t& notificationMessageSize, uint16_t& measurementMessageSize)
{
	m_driver->read(XBUS_PIPE_STATUS, status, sizeof(status));
	notificationMessageSize = status[0] | (status[1] << 8);
	measurementMessageSize = status[2] | (status[3] << 8);
}


/*!	\brief Read from notification or measurement data pipe
	\param[out] buffer Result buffer
	\param[in] size Number of bytes to read
	\param[in] pipe Pipe from which to read, XBUS_NOTIFICATION_PIPE or XBUS_MEASUREMENT_PIPE
*/
void MtsspInterface::readFromPipe(uint8_t* buffer, uint16_t size, uint8_t pipe)
{
	ASSERT(pipe == XBUS_NOTIFICATION_PIPE || pipe == XBUS_MEASUREMENT_PIPE);
	m_driver->read(pipe, buffer, size);
}


/*! \brief Sends an xbus message to the motion tracker
	\param[in] xbusMessage Pointer to xbus message which should be send
*/
void MtsspInterface::sendXbusMessage(XbusMessage const* xbusMessage)
{
	uint8_t buffer[128];
	size_t rawLength = XbusMessage_createRawMessage(buffer, xbusMessage, m_driver->busFormat());
	m_driver->writeRaw(buffer, rawLength);
}


