
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

#ifndef MTSSP_DRIVER_H
#define MTSSP_DRIVER_H

#include "xbusmessage.h"

/*!	\class MtsspDriver
	\brief Abstract interface providing the MTSSP interface with an abstraction layer to the underlying hardware bus (SPI or I2C)

*/
class MtsspDriver
{
public:

	/*!	\brief Perform a blocking write transfer
		\param[in] opcode Opcode to use
		\param[in] data Pointer to data to be written
		\param[in] dataLength Number of data bytes to write
	*/
	virtual void write(uint8_t opcode, uint8_t const* data, int dataLength) = 0;


	/*!	\brief Perform a blocking read transfer
		\param[in] opcode Opcode to use
		\param[out] data Pointer to result buffer
		\param[in] dataLength Number of data bytes to read
	*/
	virtual void read(uint8_t opcode, uint8_t* data, int dataLength) = 0;


	/*!	\brief Perform a blocking write transfer
		\param[in] data Pointer to data to be written
		\param[in] dataLength Number of data bytes to write
	*/
	virtual void writeRaw(uint8_t const* data, int dataLength) = 0;


	/*!	\brief Returns the low level bus format that must be used for transmitting messages over this hardware bus
	*/
	virtual XbusBusFormat busFormat() const = 0;
};




#endif
