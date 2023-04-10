# Software Sensor
- For hardware design see: https://github.com/larus-breeze/hw_sensor 

## Facts
- STM32CubeIDE
- STM32F407VG
- FreeRTOS
- IMU 
- GNSS 
- Static pressure 
- Differential pressure
- CAN interface
- Bluetooth via HM19 (HW-1.0)  ESP32 (HW-2.0)
- RS232 NMEA output channels

# How to use it
Some usefull git commands to work with this repository and the included submodule **command line**: 

Clone via https:

    git clone --recurse-submodules https://github.com/larus-breeze/sw_sensor
    
Clone via ssh:

    git clone --recurse-submodules git@github.com:larus-breeze/sw_sensor.git

Switch to a branch:

    git checkout branch_name --recurse-submodules
      
Pull latest changes:

    git pull --recurse-submodules

      
      
# Build Configurations
- Release used for release versions (Max. optimized, no debug info)
- Debug  used for normal development

# Flash and prepare the sensor hardware
## STM32
- Flash via USB using the STM32CubeProgrammer and a compiled binary sw_sensor.elf file from here: https://github.com/larus-breeze/sw_sensor/releases  Keep the Boot Button pressed when inserting the USB-Cable to start the STM32 in the DFU bootloader mode.
- Compile, Flash and Debug using the STM32CubeIDE with a connected ST-Link debugger

## ESP32 controller
- Flash the *.ino file below ESP32_Firmware with arduino studio and the esp32 usb connector

## Prepare an sd-card with configuration files
- put a sensor_config.txt file (example in Configuration_files) in the sd cards root directory. Adjust nick, roll, yaw in degree according to the sensors mounting orientation. All angles set to 0 (default) means an orientation so that the pilot can see the LEDs and USB connectors and Rj45 connectors and tubes are in flight direction. It is adwised to check the configured orientation by observing the heading, roll and nick angle using an ahrs display. Initially a not so correct heading is acceptable as the magnetic calibration algorithm needs some time in the air to find the calibration parameters. A 30 minutes flight with some right and left turns should be sufficient. 
- Create a directory with the name "logger" to enable logging of all measurement data with 100Hz
- Create a directory with the name "magnetic" to enable the logging of magnetic calibration events. There should be a few of these events initially are installing the sensor. 

### Additional developer option:
- Put an empty file with the name: "sensor.readings" in order to switch the serial output format from Larus NMEA syntax to pure sensor raw data values.
- Additionally put an empty file with the name "magnetic.calibration" onto the sd-card in order to start a magnetic ground calibration. This is a development feature and not required, nor advised for normal operation. 

