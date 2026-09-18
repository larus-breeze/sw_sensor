/** *****************************************************************************
 * @file    	wlan_link_handler.h
 * @brief   	ESP32 <-> STM32 SPI2 link: firmware update upload + SD card
 *              maintenance (status, delete, format). See
 *              documentation/wlan_link.md.
 * @license 	This project is released under the GNU Public License GPL-3.0

    <Larus Flight Sensor Firmware>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 **************************************************************************/

#ifndef COMMUNICATION_WLAN_LINK_HANDLER_H_
#define COMMUNICATION_WLAN_LINK_HANDLER_H_

#include <stdint.h>

void wlan_link_handler_runnable (void *);

// Called from mti1.cpp's HAL_GPIO_EXTI_Callback() for the PB1 EXTI line -
// PB1 acts as a software chip-select (see wlan_link_handler.cpp for the
// mechanism, since the STM32F407 has no SPI2_NSS alternate function on
// this pin).
void wlan_link_cs_edge (void);

#endif /* COMMUNICATION_WLAN_LINK_HANDLER_H_ */
