## Checklist before merging
- [ ] Is the most recent master version merged? Double check with git pull and git merge master.
- [ ] Does it compile without errors?
- [ ] Has someone reviewed your changes?

### Hardware test
- [ ] Flash the ESP and STM Firmware via USB and connect a GNSS antenna
- [ ] Test without a SD-Card
- [ ] Check that the Bluetooth and CAN works and transmitts data. If possible test with Hardware-version 1.0 and 2.0!
- [ ] Insert a SD-Card with the latest sensor_config.txt file from Configuration_files and an empty file named "enable.logger" Let the sensor run for ~ 30 minutes. Afterwards check that there are only two files: data file  yymmdd_hhmmss.f* and a calibration/configuration dump file: yymmdd_hhmmss.EEPROM  There shall be no crash-dump files.

