# STM32CubeIDE for headless release builds in CI - see README.md next to this file.
# Build it from your own download of the STM32CubeIDE Linux (Debian, x86_64) installer;
# the result is meant for a *private* registry only.

ARG UBUNTU=ubuntu:24.04

# Stage 1: install from the installer, then remove what a build doesn't need, including all
# SEGGER J-Link software (its license forbids redistribution). Nothing of this stage ends up
# in the final image except the cleaned /opt/st.
FROM --platform=linux/amd64 ${UBUNTU} AS install
ARG INSTALLER
ENV DEBIAN_FRONTEND=noninteractive LICENSE_ALREADY_ACCEPTED=1
COPY ${INSTALLER} /tmp/installer.sh
RUN apt-get update -qq \
 && sh /tmp/installer.sh --noexec --target /tmp/pkg \
 && apt-get install -y -qq /tmp/pkg/st-stlink-udev-rules-*.deb /tmp/pkg/st-stlink-server-*.deb /tmp/pkg/stm32cubeide-*.deb \
 && cd /opt/st/stm32cubeide_* \
 && rm -rf plugins/*externaltools.jlink* plugins/*debug.jlink* features/*jlink* \
           plugins/*externaltools.cubeprogrammer* features/*cubeprogrammer* \
 && if find /opt/st -iname '*jlink*' -o -iname '*segger*' | grep -v -e '/jre/' -e '/openocd/.*/jlink\.cfg$' | grep .; then \
      echo 'SEGGER files left in /opt/st, extend the removal above' >&2; exit 1; fi

# Stage 2: the image that gets pushed
FROM --platform=linux/amd64 ${UBUNTU}
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update -qq \
 && apt-get install -y -qq --no-install-recommends bash ca-certificates git make python3 python3-toml python3-pyelftools \
      libgtk-3-0t64 libxtst6 libxrender1 libxi6 \
 && rm -rf /var/lib/apt/lists/*
COPY --from=install /opt/st /opt/st
LABEL org.opencontainers.image.description="STM32CubeIDE (without SEGGER J-Link and STM32CubeProgrammer) for headless builds of larus-breeze/sw_sensor. Private, not for redistribution."
WORKDIR /work
