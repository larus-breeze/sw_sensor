# Browser setup page (initial ESP32 flash + WiFi label)

`larus_esp32_setup.html` is the template of the single-file page
`build/larus_sensor_esp32_v<version>.setup.html` that
`../build_firmware.py` writes on every build (and that the release
workflow attaches to each release). Usage: see "Initial flash without a
local toolchain" in `sw_esp32/README.md`.

`build_firmware.py`'s `render_setup_page()` fills two placeholders:
- `<script id="firmwareData">`: version plus the `flash1_boot.bin` (0x1000)
  and `flash2_app.bin` (0xe000) images of that build, base64 encoded, so
  the page always carries the firmware it was built with.
- `<script id="esptoolJs">`: `vendor/esptool-js.bundle.js`, rewritten from
  an ES module into a classic script that sets `window.esptooljs`. Chrome
  doesn't run module scripts (not even from a `blob:` URL) in a page opened
  via `file://`; a classic inline script does, so the page works when
  double-clicked, offline. The generated script starts with a modification
  notice and the full `vendor/esptool-js.LICENSE`, so every standalone page
  includes the license.

Opened directly, the template only offers the WiFi label part.

## Vendored esptool-js

`vendor/esptool-js.bundle.js` is `bundle.js` from the npm package
`esptool-js@0.7.0`, unmodified (Apache-2.0, `vendor/esptool-js.LICENSE`).
To update it:

    npm pack esptool-js@<version>
    tar xzf esptool-js-<version>.tgz
    cp package/bundle.js vendor/esptool-js.bundle.js
    cp package/LICENSE vendor/esptool-js.LICENSE

then bump the version above and test a flash on real hardware.
`build_firmware.py` refuses a bundle that isn't self-contained or doesn't
end in a single `export {...}`.
