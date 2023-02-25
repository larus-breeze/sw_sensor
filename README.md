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

# How to use it: 
Clone repository including the submodules using the **command line**: 

      git clone --recurse-submodules https://github.com/larus-breeze/sw_sensor
      
      git clone --recurse-submodules git@github.com:larus-breeze/sw_sensor.git

      
# Build Configurations
- Debug   (This should be used currently)
- debug timing
- release
TODO: describe which one to use?  relase e.g. disables the flight data logging, why do we want to remove this feature in the release version?
