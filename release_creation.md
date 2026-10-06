## Checklist before creating a relase
- Is the most recent master version merged? Double check with git pull and git merge master.
- Does it compile without errors?
- Has someone reviewed your changes?
- Create a tag in the format 0.0.0 e.g. git tag 0.4.0

### Hardware lab test
- Flash the ESP (if changed) and STM32 Firmware and connect a GNSS antenna
- Test without a SD-Card and verify that configuration parameters e.g. ahrs level are correct.
- Check that the Bluetooth and CAN works and transmitts data
  - Verify that a via CAN connected frontend shows reasonable data
  - Remove GNSS Antenna(s) check that Frontend and Sensor keep working without crashs.
- Insert a SD-Card with the latest sensor_config.ini file from Configuration_files and logging enabled. Let the sensor run for ~ 30 minutes. Afterwards check that there are no crashdumps and a yymmdd_hhmmss.f* and yymmdd_hhmmss.EEPROM file.
- Check that software updates from this release versions are still working and are only executed once.
- Perform a Mass Erase and flash firmware in order to test that the system is working correctly without any prior eeprom configuration.

### Flight test
- Test the release during a flight before publication. Verify that the following during and after the test flight:
  - No crashdumps on the sd-card  after at least 30min flying.
  - AHRS, Vario, Wind, Barometric/GNSS altitude and TrueAirspeed information are reasonable
  - MCready sync is working between Larus Frontend and XCSOAR.
- Perform a testflight with some right and left turns with a clean (erased) EEPROM FLASH in order to verify that the automatic magnetic calibration is working and stored to the eeprom persistently.

### Lib test
- Create binaries of the lib analyzer and add them here: https://github.com/larus-breeze/sw_tools/tree/master/larus_data/_internal
- Use https://github.com/larus-breeze/sw_tools/blob/master/analysis/plot_regression.py as an example to compare the new with the old libary.

### How to create release binaries
- The used submodule lib commit needs a tag here. https://github.com/larus-breeze/sw_algorithms_lib/tags  Create one if missing for used commit.
- Add analyzer lib binaries here: https://github.com/larus-breeze/sw_tools/tree/master/larus_data/_internal for analysis and flight_player purposes
- Create releases from master branch commits only. Merge changes first. Check that the master branch is pushed and not in a dirty state with git status
- Chose the next version number with the format x.y.z (e.g. 9.9.9) https://github.com/larus-breeze/sw_sensor/releases
- Assign a tag to the current commit e.g. with: git tag 9.9.9
- Push tag: git push origin tag 9.9.9
- Build the firmware with `python3 build_firmware_all.py` (see README.md). The build checks that the release toolchain from `sw_stm32/scripts/release_toolchain.txt` was used; do not use `--allow-other-toolchain` for releases.
- Go to https://github.com/larus-breeze/sw_sensor/releases create a new release for the tagged commit, write release notes and attach the firmware binary
- Add the line "Built with ..." that build_firmware.py prints at the end to the release notes
