# Configuration files
Files here are used to configure the GNSS module and the stm32 firmware
      
## GNSS configuration
Us the configuration file to configure the GNSS with the ublox u-center software.
- use the uBlox_M9N_75ms.txt for the single GNSS M9N module on the PCB
- use ??? for the F9P Differential GNSS heading module

## Firmware configuration and orientation
Use the sensor_config.txt file as a template.
- adjust SensTilt_Roll, SensTilt_Nick, SensTilt_Yaw to match the mounting orientation.
- Configure the chosen GNSS module by setting GNSS_CONFIG to 1.0 for the single GNSS M9N module, 2.0 for the F9P DGNSS module.
- put this file on the micro sd card and restart the sensor to read the configuration.

