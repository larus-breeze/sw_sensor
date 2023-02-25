# Firmware Sensor
- Firmware for https://github.com/larus-breeze/hw_sensor 

## Facts
- STM32CubeIDE
- STM32F407VG
- FreeRTOS
- IMU 
- GNSS 
- Static pressure 
- Differential pressure
- CAN interface
- Bluetooth
- RS232

# How to use it
Clone repository including the submodules using the **command line**: 

      git clone --recurse-submodules https://github.com/larus-breeze/sw_sensor
      
      git clone --recurse-submodules git@github.com:larus-breeze/sw_sensor.git

      
# Build Configurations
- Debug   (This should be used currently)
- debug timing
- release
TODO: describe which one to use?  relase e.g. disables the flight data logging, why do we want to remove this feature in the release version?


# Flash and prepare the sensor hardware
## STM32
- Flash via USB using the STM32CubeProgrammer and a compiled binary sw_sensor.elf file from here: https://github.com/larus-breeze/sw_sensor/releases  Keep the Boot Button pressed when inserting the USB-Cable to start the STM32 in the DFU bootloader mode.
- Compile, Flash and Debug using the STM32CubeIDE with a connected ST-Link debugger

## ESP32 controller
- Flash the *.ino file below ESP32_Firmware with arduino studio and the esp32 usb connector

## Prepare an sd-card with configuration files
- put a sensor_config.txt file (example in Configuration_files) TODO: describe how to configure the sensor parameters in the sensor_config.txt file
- put an empty "enable.logger" file on the sd card to enable logging the measurement data with 100Hz
- TODO: describe the features by putting empty sensor.readings and magnetic.calibration files on the sd-card.
