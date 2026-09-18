# scripts
Scripts for creating header files with git commit and tag version information
and to create binaries for stm32 sensor firmware updates via an uSD card.

## Python environment setup
`build_firmware.py`/`create-git-info-header.py` use only the Python
standard library; `pack.py` additionally needs `toml` and `pyelftools`,
listed in `requirements.txt`. From `sw_stm32/`:

    python3 -m venv .venv
    source .venv/bin/activate   # Windows: .venv\Scripts\activate
    pip install -r scripts/requirements.txt

(Building both sides together via the repository root's
`build_firmware_all.py`? Use the root `requirements.txt` instead, which
pulls in this file plus the ESP32 side's.)

## Version information
- create-git-info-header.py inserts the git commit and tag version information
into a c header for the compile process and into the toml files which are used
to create firmware images.

## Building the firmware
- Either build it in STM32CubeIDE itself (Project -> Build Project, with the
"Release" build configuration active), or headlessly from the command line
with build_firmware.py:

      python3 scripts/build_firmware.py

  This drives STM32CubeIDE's headless build mode to compile the Release
  configuration without opening the IDE - so it can't accidentally build
  against whatever configuration (e.g. Debug) the IDE happened to have
  selected - and then runs pack.py automatically to also produce the SD-card
  update image. Requires STM32CubeIDE
  (https://www.st.com/en/development-tools/stm32cubeide.html) to be
  installed; the headless launcher is auto-detected in common install
  locations, or pass --cubeide /path/to/stm32cubeide (or set the
  STM32CUBEIDE environment variable) to point at a non-standard one. Pass
  --legacy to also produce the LEGACY image, or --no-pack to only build and
  skip packaging. Run with --help for all options.

- Either way, make sure the *.elf binary this produces in the Release
directory is actually the one you intend to package - pack.py now warns if
it's older than 10 minutes.

- build_firmware.py also mirrors the raw build output (*.elf, *.map, ...) from
the Release directory into `sw_stm32/build/` (gitignored), for symmetry with
the ESP32 side's own raw `arduino-cli` output directory (`sw_esp32/build/`).

- It also copies the *.elf as `larus_sensor_stm32_v<version>.elf` into `build/`
at the repository root (gitignored, alongside pack.py's own packaged image
below) - STM32CubeProgrammer reads *.elf natively (it extracts the loadable
segments' own addresses from the ELF itself), so this is flashable directly
over USB DFU, no separate raw *.bin needed for that path.

## Creating binaries for uSD card updates
The script pack.py creates the binary files which can be used via uSD card to
update the sensors stm32 firmware. Pass the optional parametern LEGACY to create
a firmware binary which can be used to update older sensor firmware versions.
up to 0.4.0. build_firmware.py runs this automatically after building; invoke
it directly (python3 scripts/pack.py or python3 scripts/pack.py LEGACY) to
just re-package an *.elf that's already been built.

The packaged image is written to `build/` at the repository root
(gitignored), alongside the ESP32 side's own build_firmware.py output.

