## Checklist before merging
- [ ] Is the most recent master version merged? Double check with git pull and git merge master.
- [ ] Does it compile without errors?
- [ ] Has someone reviewed your changes?

### Hardware test
- [ ] Flash the ESP (if changed) and STM32 Firmware and connect a GNSS antenna
- [ ] Test without a SD-Card
- [ ] Check that the Bluetooth and CAN works and transmitts data
- [ ] Insert a SD-Card with the latest sensor_config.ini file from Configuration_files and logging enabled. Let the sensor run for ~ 30 minutes. Afterwards check that there are no crashdumps and a yymmdd_hhmmss.f* and yymmdd_hhmmss.EEPROM file.


