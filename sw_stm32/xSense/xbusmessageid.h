
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

#ifndef XBUSMESSAGEID_H
#define XBUSMESSAGEID_H


/*! \brief Xbus message IDs. */
enum XsMessageId
{
	XMID_Wakeup              = 0x3E,
	XMID_WakeupAck           = 0x3F,
	XMID_ReqDid              = 0x00,
	XMID_DeviceId            = 0x01,
	XMID_GotoConfig          = 0x30,
	XMID_GotoConfigAck       = 0x31,
	XMID_GotoMeasurement     = 0x10,
	XMID_GotoMeasurementAck  = 0x11,
	XMID_MtData2             = 0x36,
	XMID_ReqOutputConfig     = 0xC0,
	XMID_SetOutputConfig     = 0xC0,
	XMID_OutputConfig        = 0xC1,
	XMID_Reset               = 0x40,
	XMID_ResetAck            = 0x41,
	XMID_Error               = 0x42,
	XMID_ToggleIoPins        = 0xBE,
	XMID_ToggleIoPinsAck     = 0xBF,
	XMID_FirmwareUpdate      = 0xF2,
	XMID_GotoBootLoader      = 0xF0,
	XMID_GotoBootLoaderAck   = 0xF1,
	XMID_ReqFirmwareRevision = 0x12,
	XMID_FirmwareRevision    = 0x13


};



#endif
