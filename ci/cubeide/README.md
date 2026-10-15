# STM32CubeIDE image for release builds in CI

`.github/workflows/release-build.yml` builds the firmware with STM32CubeIDE
itself (headless, `sw_stm32/scripts/build_firmware.py`) on `master`, on tags
and on demand, and keeps the ELF and the SD-card update image as workflow
artifacts. It runs in a container image with STM32CubeIDE, which a maintainer
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

   Stage 1 installs STM32CubeIDE and removes J-Link and STM32CubeProgrammer;
   the final image only contains the cleaned `/opt/st` plus the packages
   `pack.py` needs (about 0.9 GB). Neither the installer nor removed files
   end up in a layer of the final image.
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

## Checked
With STM32CubeIDE 2.2.0 (GNU Tools for STM32 14.3.rel1), `master` at
`9bdcc91` builds in the container in about a minute (0 errors), including
the SD-card update image. Compared with `ci_build.py` and Arm GNU Toolchain
14.3.rel1 for the same commit, the RAM sections and the compiler warnings
are identical and flash differs by 0.07%.
