# STM32CubeIDE image for release builds in CI

`.github/workflows/release-build.yml` builds the firmware with STM32CubeIDE
itself (headless, `sw_stm32/scripts/build_firmware.py`) on `master`, on
version tags (x.y.z) and on demand, together with the ESP32 firmware
(esp32:esp32 core pinned in the workflow), and keeps the images as workflow
artifacts. For a version tag on a `master` commit whose library commit is
tagged in sw_algorithms_lib, it also creates a draft release with the images,
SHA-256 checksums and the toolchain versions (see `release_creation.md`).

The STM32 job runs in a container image with STM32CubeIDE, which a maintainer
builds from their own STM32CubeIDE download and keeps in a **private**
registry. The pull-request checks don't need it: they use
`sw_stm32/scripts/ci_build.py` with the Arm GNU Toolchain (see
`sw_stm32/scripts/README.md`).

## Why private
The image contains STM32CubeIDE, so pushing it is redistribution:
- **SEGGER J-Link software** is part of STM32CubeIDE, and its license forbids
  redistribution. The Dockerfile removes it, together with
  STM32CubeProgrammer, which bundles `libjlinkarm.so`. Neither is needed to
  build. The build fails if any J-Link/SEGGER file is left.
- **ST's license and the open-source parts** (GCC, newlib, Java, Eclipse,
  LGPL libraries; see "Additional License Terms for STM32CubeIDE") come with
  conditions for redistribution, such as license texts and source offers.
  ST's license also restricts use to ST devices, and STM32CubeIDE is
  downloaded through myST, which includes export-control checks.

A private image that only this repository's workflows can pull keeps it
within the project. Publishing it would need these points to be cleared
first, ideally with ST.

## Build and push the image (maintainer)
Requires Docker on an x86_64 machine (or Apple silicon with Rosetta for
amd64 containers; QEMU emulation tends to crash STM32's GCC).

1. Download the STM32CubeIDE installer for **Linux, Debian/Ubuntu, x86_64**
   from st.com (myST login, accept the license), e.g.
   `stm32cubeide_2.2.0_29186_20260626_0934-Lin-Deb-x86_64.sh`, and put it
   into this directory (`.gitignore` keeps it out of git).
2. Build:

       docker build --platform linux/amd64 \
           --build-arg INSTALLER=stm32cubeide_2.2.0_29186_20260626_0934-Lin-Deb-x86_64.sh \
           -t ghcr.io/larus-breeze/stm32cubeide:2.2.0 .

   Stage 1 installs STM32CubeIDE and removes J-Link, STM32CubeProgrammer and
   everything a headless build doesn't need: the debug device databases,
   CMake, OpenOCD, documentation, GDB, and the GCC libraries of all CPU
   variants except the ones listed in the build argument `MULTILIBS`
   (default: Cortex-M4F as in this firmware and the RM3100 magnetometer
   module, Cortex-M3 as in sw_audio_box). The final image only contains the
   cleaned `/opt/st` (about 0.8 GB) plus the packages `pack.py` needs, about
   490 MB compressed, which fits a free GitHub account's 500 MB of private
   package storage. Neither the installer nor removed files end up in a layer
   of the final image.
3. Push it to the GitHub Container Registry with a personal access token
   that has `write:packages`:

       echo $TOKEN | docker login ghcr.io -u <github-user> --password-stdin
       docker push ghcr.io/larus-breeze/stm32cubeide:2.2.0

4. On GitHub, open the package (larus-breeze → Packages →
   `stm32cubeide`):
   - keep the visibility **private**;
   - under "Manage Actions access", give the repository `sw_sensor` read
     access.

## Updating STM32CubeIDE
Build and push a new tag (e.g. `stm32cubeide:2.3.0`) from the new installer,
then change the image tag in `.github/workflows/release-build.yml` and the
release toolchain in `sw_stm32/scripts/release_toolchain.txt` (#258) together.
The ESP32 core of releases is `ESP32_CORE` in the same workflow; keep it in
line with the core that `firmware-build.yml` checks pull requests with.

## Checked
With STM32CubeIDE 2.2.0 (GNU Tools for STM32 14.3.rel1), `master` at
`9bdcc91` builds in the container in about a minute (0 errors), including
the SD-card update image. The slimmed image produces the same firmware as a
full installation (identical section sizes). Compared with `ci_build.py` and Arm GNU Toolchain
14.3.rel1 for the same commit, the RAM sections and the compiler warnings
are identical and flash differs by 0.07%.
