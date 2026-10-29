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
           plugins/*externaltools.openocd* plugins/*debug.openocd* \
 && if find /opt/st -iname '*jlink*' -o -iname '*segger*' | grep -v -e '/jre/' | grep .; then \
      echo 'SEGGER files left in /opt/st, extend the removal above' >&2; exit 1; fi

# Not needed for headless builds, removed to keep the image small: debug device databases,
# CMake/OpenOCD tools, documentation, GDB, and the GCC libraries of all CPU variants except
# the ones in MULTILIBS (default: Cortex-M4F as in sw_sensor and the RM3100 module,
# Cortex-M3 as in sw_audio_box).
ARG MULTILIBS="-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb;-mcpu=cortex-m3 -mthumb"
RUN cd /opt/st/stm32cubeide_* \
 && rm -rf plugins/*productdb.debug* plugins/*externaltools.cmake* \
           plugins/*.documentation_* plugins/org.eclipse.cdt.doc.* \
 && cd plugins/*gnu-tools-for-stm32*linux64*/tools \
 && keep=$(echo "$MULTILIBS" | tr ';' '\n' | while read -r f; do bin/arm-none-eabi-gcc $f -print-multi-directory; done | sort -u) \
 && echo "keeping multilibs: $keep" \
 && for base in arm-none-eabi/lib lib/gcc/arm-none-eabi/*; do \
      for d in $(cd $base && ls -d thumb/*/* arm/*/* 2>/dev/null); do \
        echo "$keep" | grep -qx "$d" || rm -rf "$base/$d"; \
      done; \
    done \
 && rm -rf share/doc share/info share/man share/gdb bin/arm-none-eabi-gdb* bin/arm-none-eabi-lto-dump \
 && du -sh /opt/st

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
