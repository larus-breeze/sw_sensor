
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

#include "application.h"
//#include "init_gpio.h"
//#include "board.h"
#include "mtssp_interface.h"
#include "xbusmessageid.h"
//#include "wait.h"
#include "xbusdef.h"
#include <string>
#include "xbustostring.h"
#include "xbushelpers.h"

/*!	\class Application
	The main application class of 'example_mti1_i2c_spi_receive_measurement_data'
*/


/*!	\brief Constructs an Application
	\param hostInterface The HostInterface for communicating with the PC
	\param driver The MtsspDriver for communicating with the MTi
*/
Application::Application(HostInterface* hostInterface, MtsspDriver* driver)
	: m_host(hostInterface)
	, m_driver(driver)
	, m_state(STATE_Idle)
{
	m_device = new MtsspInterface(m_driver);
}


/*!	\brief Returns the value of the DataReady line
*/
bool checkDataReadyLine()
{
	return HAL_GPIO_ReadPin(DATA_READY_PORT, DATA_READY_PIN) == GPIO_PIN_SET;
}


/*!	\brief Defines the main loop of the program which handles user commands
*/
void Application::run()
{
	handleEvent(EVT_Start);

	while (true)
	{
		if (checkDataReadyLine())
		{
			readDataFromDevice();
		}

		if (m_host->kbhit())
		{
			char c = m_host->getch();
			m_host->printf("\n");

			if (c == 'h')
			{
				printHelpText();
			}

			if (c == 'c')
			{
				handleEvent(EVT_GotoConfig);
			}

			if (c == 'm')
			{
				handleEvent(EVT_GotoMeasuring);
			}

			if (c == 'd')
			{
				handleEvent(EVT_RequestDeviceId);
			}

			if (c == 'R')
			{
				resetDevice();
			}
		}
	}
}


/*!	\brief Prints the help text to the console
*/
void Application::printHelpText()
{
/*
	m_host->printf("\n");
	m_host->printf("(h) help:     Print this help text\n");
	m_host->printf("(R) reset:    Reset the device\n");
	m_host->printf("(c) config:   Goto config mode\n");
	m_host->printf("(m) measure:  Goto measurement mode\n");
	m_host->printf("(d) deviceid: Request device id\n");
	m_host->printf("\n");
*/
}


/*!	\brief Implements the state machine which defines the program
*/
void Application::handleEvent(Event event, const uint8_t* data)
{
	switch (m_state)
	{
		case STATE_Idle:
		{
			if (event == EVT_Start)
			{
				m_host->printf("Resetting the device\n");
				resetDevice();
				m_state = STATE_WaitForWakeUp;
			}
		} break;

		case STATE_WaitForWakeUp:
		{
			if (event == EVT_XbusMessage && Xbus::getMessageId(data) == XMID_Wakeup)
			{
				m_host->printf("Got Wake-up from device\n");
				XbusMessage msg(XMID_GotoConfig);
				m_device->sendXbusMessage(&msg);
				m_state = STATE_WaitForConfigMode;
			}
		} break;

		case STATE_WaitForConfigMode:
		{
			XbusMessage msg(XMID_ReqDid);
			m_device->sendXbusMessage(&msg);
			m_state = STATE_WaitForDeviceId;
		} break;

		case STATE_WaitForDeviceId:
		{
			if (event == EVT_XbusMessage && Xbus::getMessageId(data) == XMID_DeviceId)
			{
				m_host->printf("Got DeviceId\n");
				XbusMessage msg(XMID_ReqFirmwareRevision);
				m_device->sendXbusMessage(&msg);
				m_state = STATE_WaitForFirmwareRevision;
			}
		} break;

		case STATE_WaitForFirmwareRevision:
		{
			if (event == EVT_XbusMessage && Xbus::getMessageId(data) == XMID_FirmwareRevision)
			{
				m_host->printf("Got firmware revision\n");
				uint8_t xbusData[] = {0x20, 0x30, 0x00, 0x0A};	// (Output mode: Euler angles (0x2030) at 10 Hz)
				XbusMessage msg(XMID_SetOutputConfig);
				msg.m_length = sizeof(xbusData);
				msg.m_data = xbusData;
				m_device->sendXbusMessage(&msg);
				m_state = STATE_WaitForSetOutputConfigurationAck;
			}
		} break;

		case STATE_WaitForSetOutputConfigurationAck:
		{
			if (event == EVT_XbusMessage && Xbus::getMessageId(data) == XMID_OutputConfig)
			{
				m_host->printf("Output configuration written to device\n");
				printHelpText();
				m_state = STATE_Ready;
			}
		} break;

		case STATE_Ready:
		{
			if (event == EVT_GotoConfig)
			{
				XbusMessage msg(XMID_GotoConfig);
				m_device->sendXbusMessage(&msg);
			}

			if (event == EVT_GotoMeasuring)
			{
				XbusMessage msg(XMID_GotoMeasurement);
				m_device->sendXbusMessage(&msg);
			}

			if (event == EVT_RequestDeviceId)
			{
				XbusMessage msg(XMID_ReqDid);
				m_device->sendXbusMessage(&msg);
			}

			if (event == EVT_XbusMessage)
			{
				const char* s = xbusToString(data);
				m_host->printf("%s\n", s);
			}
		} break;
	}
}


/*!	\brief Resets the MTi
*/
void Application::resetDevice()
{
	HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_SET);
	wait_us(1000);
	HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_RESET);
}


/*!	\brief Read data from the Notification and Control pipes of the device
*/
void Application::readDataFromDevice()
{
	uint16_t notificationMessageSize;
	uint16_t measurementMessageSize;
	m_device->readPipeStatus(notificationMessageSize, measurementMessageSize);

	m_dataBuffer[0] = XBUS_PREAMBLE;
	m_dataBuffer[1] = XBUS_MASTERDEVICE;

	if (notificationMessageSize && notificationMessageSize < sizeof(m_dataBuffer))
	{
		m_device->readFromPipe(&m_dataBuffer[2], notificationMessageSize, XBUS_NOTIFICATION_PIPE);
		handleEvent(EVT_XbusMessage, m_dataBuffer);
	}

	if (measurementMessageSize && measurementMessageSize < sizeof(m_dataBuffer))
	{
		m_device->readFromPipe(&m_dataBuffer[2], measurementMessageSize, XBUS_MEASUREMENT_PIPE);
		handleEvent(EVT_XbusMessage, m_dataBuffer);
	}
}


