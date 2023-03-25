
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

#include "xbusmessage.h"
#include "xbusdef.h"

/*!	\brief Format a message into the raw mtssp format ready for transmission to a motion tracker.
*/
size_t XbusMessage_createRawMessage(uint8_t* dest, struct XbusMessage const* message, enum XbusBusFormat format)
{
	int n;
	uint8_t checksum;
	uint16_t length;
	uint8_t* dptr = dest;

	if (dest == 0)
	{
		switch (format)
		{
		case XBF_I2c:
			return (message->m_length < 255) ? message->m_length + 4 : message->m_length + 6;
		case XBF_Spi:
			return (message->m_length < 255) ? message->m_length + 7 : message->m_length + 9;
		case XBF_Uart:
			return (message->m_length < 255) ? message->m_length + 5 : message->m_length + 7;
		}
	}

	switch (format)
	{
	case XBF_I2c:
		*dptr++ = XBUS_CONTROL_PIPE;
		break;

	case XBF_Spi:
		*dptr++ = XBUS_CONTROL_PIPE;
		// Fill bytes required to allow MT to process data
		*dptr++ = 0;
		*dptr++ = 0;
		*dptr++ = 0;
		break;

	case XBF_Uart:
		*dptr++ = XBUS_PREAMBLE;
		*dptr++ = XBUS_MASTERDEVICE;
		break;
	}

	checksum = 0;
	checksum -= XBUS_MASTERDEVICE;

	*dptr = message->m_mid;
	checksum -= *dptr++;

	length = message->m_length;

	if (length < XBUS_EXTENDED_LENGTH)
	{
		*dptr = length;
		checksum -= *dptr++;
	}
	else
	{
		*dptr = XBUS_EXTENDED_LENGTH;
		checksum -= *dptr++;
		*dptr = length >> 8;
		checksum -= *dptr++;
		*dptr = length & 0xFF;
		checksum -= *dptr++;
	}

	for (n = 0; n < message->m_length; n++)
	{
		*dptr = message->m_data[n];
		checksum -= *dptr++;
	}

	*dptr++ = checksum;

	return dptr - dest;
}
