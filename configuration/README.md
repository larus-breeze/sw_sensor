# Configuration files
The files here are used to configure the GNSS module and the configuration parameters.
      
## GNSS configuration
Use the configuration file(s) to configure the GNSS modules with the ublox u-center software. Save the configuration to the modules internal ROM to ensure it is not lost after a power cycle.
- use the uBlox_M9N_75ms.txt for the single GNSS M9N module soldered to the PCB
- use Ardusimple_.*.txt for a F9P differential GNSS base and heading module
    - Ensure to use the F9P Firmware 1.13 which supports heading at 10Hz. Use the File UBX_F9_100_HPG_113_ZED_F9P.7e6e899c5597acddf2f5f2f70fdf5fbe.bin or from Download https://www.ardusimple.com/how-to-configure-ublox-zed-f9p
    - The F9P shall output binary data UBX-RELPOSNED and UBX-PVT with 10 Hz at 115200 baud. Expected format is here: https://github.com/larus-breeze/sw_algorithms_lib/blob/main/NAV_Algorithms/GNSS.h
- use larus_config_X20D.ucf for the u-blox ZED-X20D dual-antenna heading module
    - Load it with u-center 2 version 26.05.3 or later. The module needs firmware HDG 2.00 (check with UBX-MON-VER).
    - The file stores its Larus settings in RAM, BBR and flash: UBX-NAV-PVT and UBX-NAV-DAHEADING on UART1 with 10 Hz at 115200 baud, no NMEA on UART1, dynamic model "airborne < 4 g". Its other entries only set the RAM and are lost after a power cycle.

## Sensor configuration
In case the sensor is used standalone without a Larus Frontend the larus_sensor_config.ini configuration file shall be used to configure the system parameters. Use the provided file here as a template and modify the values. Put the file on to the micro sd card and restart the sensor to update the configuration to the internal eeprom. The file will be renamed after beeing processed in order to apply it only once. It is recommended to use a Larus Frontend Display (if available) to configure these parameters, but it can also be done manually.

### Configuration Parameter Description

#### Mounting orientation
Describe the mounting orientation by adjusting the following parameters in [deg]. Verify the orientation by checking the AHRS output e.g. via XCSOAR. All angles set to 0 (default) means an orientation so that the pilot can see the LEDs and USB connectors and Rj45 connectors and tubes are in flight direction.
 - SensTilt_Roll
 - SensTilt_Pitch
 - SensTilt_Yaw

#### GNSS Module
Configure the chosen GNSS module by setting GNSS_CONFIG parameter to
- 1.0 for the single GNSS M9N module
- 2.0 for the F9P D-GNSS module with 2 Antennas
- 3.0 for an F9P with a separate F9H heading module
- 4.0 for the X20D D-GNSS module with 2 Antennas

#### D-GNSS-configuration
The following parameters are necessary if the system is equipped with a D-GNSS receiver with two antennas. The system requires information on how far apart the antennas are from each other and how large the lateral and height offset is.
- ANT_BASELEN:  Distance between the two antennas along the aircraft's longitudinal axis in [m]. The value should be positive, as the secondary antenna should be mounted behind the main antenna.
- ANT_SLAVE_DOWN:  Height difference of the secondary antenna in [m]. The value must be positive if it is lower than the main antenna.
- ANT_SLAVE_RIGHT: Lateral offset in the direction of the pitch axis between the main and secondary antennas in [m]. The value must be positive if the secondary antenna is further to the right in the direction of flight.

#### Horizon Attitude Data
Some competitions do not allow to use a system which provides AHRS Horizon data. Thus the ouput of this data (via NMEA and CAN) can be disabled via the Horizon_active parameter. Set it to
- 0.0 for disabled horizon attitude data (Competition mode)
- 1.0 for enabled horizon attitude data (Default value)

