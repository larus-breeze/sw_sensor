## Checklist before creating a relase
- Is the most recent master version merged? Double check with git pull and git merge master.
- Does it compile without errors?
- Has someone reviewed your changes?
- Create a tag in the format 0.0.0 e.g. git tag 0.4.0

### Hardware lab test
- Flash the ESP (if changed) and STM32 Firmware and connect a GNSS antenna
- Test without a SD-Card
- Check that the Bluetooth and CAN works and transmitts data
- Insert a SD-Card with the latest sensor_config.ini file from Configuration_files and logging enabled. Let the sensor run for ~ 30 minutes. Afterwards check that there are no crashdumps and a yymmdd_hhmmss.f* and yymmdd_hhmmss.EEPROM file.
- Check that software updates from this release versions are still working and are only executed once.

### Flight test
- Test the release during a flight before publication. Verify that the following during and after the test flight:
  - No crashdumps on the sd-card  after at least 30min flying.
  - AHRS, Vario, Wind, Barometric/GNSS altitude and TrueAirspeed information are reasonable
