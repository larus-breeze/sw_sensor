# Configuration files
Files here are used to configure the GNSS module and the stm32 firmware
      
## GNSS configuration
Use the configuration file(s) to configure the GNSS modules with the ublox u-center software
- use the uBlox_M9N_75ms.txt for the single GNSS M9N module soldered to the PCB
- use Ardusimple_.*.txt for a F9P differential GNSS base and heading module
    - Ensure to use the F9P Firmware which supports heading at 10Hz: https://www.u-blox.com/en/ubx-viewer/view/UBX_F9_100_HPG_113_ZED_F9P.7e6e899c5597acddf2f5f2f70fdf5fbe.bin?url=https%3A%2F%2Fwww.u-blox.com%2Fsites%2Fdefault%2Ffiles%2FUBX_F9_100_HPG_113_ZED_F9P
.7e6e899c5597acddf2f5f2f70fdf5fbe.bin
    - The F9P shall output binary data UBX-RELPOSNED and UBX-PVT with 10 Hz at 115200 baud. Expected format is here: https://github.com/larus-breeze/sw_algorithms_lib/blob/main/NAV_Algorithms/GNSS.h

## Firmware configuration and orientation
Use the sensor_config.txt file as a template.
- describe the mounting orientation by adjusting the following parameters in [deg]. Verify the orientation by checking the AHRS output e.g. via OpenSoar. All angles set to 0 (default) means an orientation so that the pilot can see the LEDs and USB connectors and Rj45 connectors and tubes are in flight direction.
    - SensTilt_Roll
    - SensTilt_Nick
    - SensTilt_Yaw
- Configure the chosen GNSS module by setting GNSS_CONFIG parameter to:
    - 1.0 for the single GNSS M9N module 
    - 2.0 for the F9P DGNSS module
- Put the sensor_config.txt file on the micro sd card and restart the sensor to update the configuration to the internal eeprom.

