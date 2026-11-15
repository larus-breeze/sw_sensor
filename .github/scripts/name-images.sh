#!/bin/bash
# Copies the images of a CI build to images/, named after the commit like
# GIT_COMMIT_HASH in the firmware, and lists them in the run summary.
# Usage: name-images.sh stm32|esp32 [suffix]
set -euo pipefail
target=$1 suffix=${2:-}
sha=$(git log -1 --format=%h | cut -c1-7)
echo "sha=$sha" >> "$GITHUB_OUTPUT" # first, so the logs are uploaded under this name after a failed build
mkdir -p images
case $target in
  stm32)
    cp build/larus_sensor_stm32_v*.bin "images/larus_sensor_stm32-$sha$suffix.bin"
    cp sw_stm32/Release/sw_sensor.elf "images/larus_sensor_stm32-$sha$suffix.elf" ;;
  esp32)
    for kind in .flash1_boot .flash2_app; do
      mv build/larus_sensor_esp32_v*$kind.bin "images/larus_sensor_esp32-$sha$kind.bin"
    done
    mv build/larus_sensor_esp32_v*.bin "images/larus_sensor_esp32-$sha.bin"
    mv build/larus_sensor_esp32_v*.setup.html "images/larus_sensor_esp32-$sha.setup.html" ;;
esac
{
  echo "### Test build of $sha, not a release"
  [ -n "${PR_HEAD:-}" ] && echo "Merge of the pull request (${PR_HEAD::7}) into its base. It contains code that isn't reviewed yet."
  [ -n "$suffix" ] && echo "Built with the Arm GNU Toolchain, not with STM32CubeIDE like the releases."
  echo
  ls images | sed 's/^/- `/; s/$/`/'
} >> "$GITHUB_STEP_SUMMARY"
