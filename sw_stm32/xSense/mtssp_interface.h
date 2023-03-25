
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

#ifndef MTSSP_INTERFACE_H
#define MTSSP_INTERFACE_H
#include <stdint.h>
#include "mtssp_driver.h"


#define DRDY_CONFIG_MEVENT_POS	3	//!	\brief Measurement pipe DataReady event enable: 0 = Disabled, 1 = Enabled
#define DRDY_CONFIG_NEVENT_POS	2	//!	\brief Notification pipe DataReady event enable: 0 = Disabled, 1 = Enabled
#define DRDY_CONFIG_OTYPE_POS	1	//!	\brief Output type DataReady pin: 0 = Push/pull, 1 = Open drain
#define DRDY_CONFIG_POL_POS		0	//!	\brief Polarity DataReady pin: 0 = Idle low, 1 = Idle high


class MtsspInterface
{
	public:
		MtsspInterface(MtsspDriver* driver);

		void readProtocolInfo(uint8_t& version, uint8_t& dataReadyConfig);
		void configureProtocol(uint8_t dataReadyConfig);
		void readPipeStatus(uint16_t& notificationMessageSize, uint16_t& measurementMessageSize);
		void readFromPipe(uint8_t* buffer, uint16_t size, uint8_t pipe);
		void sendXbusMessage(XbusMessage const* xbusMessage);

	private:
		MtsspDriver* m_driver;
};



#endif
