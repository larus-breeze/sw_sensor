# Configuration files
Files here are used to configure the GNSS module and the stm32 firmware
      
## GNSS configuration
Use the configuration file(s) to configure the GNSS modules with the ublox u-center software. Save the configuration to the modules internal ROM to ensure it is not lost after a power cycle.
- use the uBlox_M9N_75ms.txt for the single GNSS M9N module soldered to the PCB
- use Ardusimple_.*.txt for a F9P differential GNSS base and heading module
    - Ensure to use the F9P Firmware 1.13 which supports heading at 10Hz. Use the File UBX_F9_100_HPG_113_ZED_F9P.7e6e899c5597acddf2f5f2f70fdf5fbe.bin or from Download https://www.ardusimple.com/how-to-configure-ublox-zed-f9p
    - The F9P shall output binary data UBX-RELPOSNED and UBX-PVT with 10 Hz at 115200 baud. Expected format is here: https://github.com/larus-breeze/sw_algorithms_lib/blob/main/NAV_Algorithms/GNSS.h

## Firmware configuration and orientation
Use the larus_sensor_config.ini file as a template.
- describe the mounting orientation by adjusting the following parameters in [deg]. Verify the orientation by checking the AHRS output e.g. via OpenSoar. All angles set to 0 (default) means an orientation so that the pilot can see the LEDs and USB connectors and Rj45 connectors and tubes are in flight direction.
    - SensTilt_Roll
    - SensTilt_Pitch
    - SensTilt_Yaw
- Configure the chosen GNSS module by setting GNSS_CONFIG parameter to:
    - 1.0 for the single GNSS M9N module 
    - 2.0 for the F9P D-GNSS module
- Put the larus_sensor_config.ini file on the micro sd card and restart the sensor to update the configuration to the internal eeprom. Remove the *.ini file after use.

