#!/usr/bin/env python3
"""Generate a printable WiFi-AP label (QR code + text) for a Larus sensor.

Encodes the standard WiFi-network QR payload
("WIFI:S:<ssid>;T:WPA;P:<password>;;") so a phone can join the sensor's
access point by scanning the printed label, matching the scheme described
in documentation/wlan_link.md ("AP identity: SSID and password").

IMPORTANT - there is deliberately no "--mac" mode that recomputes the
password from a MAC address the way sw_esp32/wlan_link/ap_identity.cpp
does on the device. Doing that correctly requires knowing the exact byte
order ESP.getEfuseMac() packs the 48-bit MAC into its uint64_t return value
in, relative to the MAC address string convention (e.g. from `esptool.py
read_mac` or WiFi.macAddress()) - and that couldn't be confirmed without
real ESP32 hardware to test against. Silently guessing at that would risk
printing labels with the *wrong* password with no way to notice until
someone tries to scan one. Instead, this script only ever renders a QR
code from the exact SSID/password the firmware itself already computed and
printed over its serial console at boot (see wlan_link.ino's
setup()) - either pasted in directly, or read live from the serial port -
which is correct by construction since nothing is recomputed independently.

Usage:
    # from values already read off the serial monitor:
    python3 scripts/generate_ap_label.py --ssid Larus_1A2B --password ABCDEFGH23456789

    # read them directly from a freshly flashed/reset board:
    python3 scripts/generate_ap_label.py --serial-port /dev/ttyUSB0

Without --output, the label is written to build/larus_wifi_label_<ssid>.png
(repository root) - the SSID in the filename so labels for several devices
don't overwrite each other. Pass --output to write somewhere else instead.

Requires the 'qrcode' and 'Pillow' packages (pip install qrcode[pil]), and
additionally 'pyserial' (pip install pyserial) for --serial-port - see
requirements.txt in this directory.
"""

import argparse
import re
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT_DIR = SCRIPT_DIR.parent.parent

try:
    import qrcode
except ImportError:
    sys.exit("Missing dependency 'qrcode'. Install with: pip install qrcode[pil]")

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    sys.exit("Missing dependency 'Pillow'. Install with: pip install Pillow")


SSID_LINE_RE = re.compile(r"WiFi AP SSID:\s*(\S+)")
PASSWORD_LINE_RE = re.compile(r"WiFi AP password:\s*(\S+)")

CANDIDATE_FONT_PATHS = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    "/Library/Fonts/Arial.ttf",  # macOS
    "C:\\Windows\\Fonts\\arial.ttf",  # Windows
]


def load_font(size):
    for path in CANDIDATE_FONT_PATHS:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    # Always works, even with no system fonts found - small bitmap font.
    return ImageFont.load_default(size=size)


def build_wifi_qr_payload(ssid, password):
    """Must match ap_identity.cpp's buildWifiQrPayload() exactly."""
    return f"WIFI:S:{ssid};T:WPA;P:{password};;"


def read_ssid_password_from_serial(port, baud=115200, timeout=15):
    try:
        import serial
    except ImportError:
        sys.exit("Missing dependency 'pyserial' for --serial-port. Install with: pip install pyserial")

    print(f"Opening {port} @ {baud} baud, resetting the board to capture its boot output...")
    with serial.Serial(port, baud, timeout=1) as ser:
        # Standard ESP32 auto-reset sequence (same one esptool/Arduino IDE
        # use): toggle DTR/RTS to pulse the EN pin low and back.
        ser.dtr = False
        ser.rts = True
        time.sleep(0.1)
        ser.rts = False
        time.sleep(0.1)

        ssid = None
        password = None
        deadline = time.time() + timeout
        buffer = b""
        while time.time() < deadline and (ssid is None or password is None):
            chunk = ser.read(256)
            if not chunk:
                continue
            buffer += chunk
            text = buffer.decode("utf-8", errors="replace")
            if ssid is None:
                m = SSID_LINE_RE.search(text)
                if m:
                    ssid = m.group(1)
            if password is None:
                m = PASSWORD_LINE_RE.search(text)
                if m:
                    password = m.group(1)

    if ssid is None or password is None:
        sys.exit(
            "Did not see both 'WiFi AP SSID: ...' and 'WiFi AP password: ...' "
            f"lines within {timeout}s. Check the port/baud rate, or paste the "
            "values manually with --ssid/--password instead."
        )
    return ssid, password


def build_label_image(ssid, password, ip):
    payload = build_wifi_qr_payload(ssid, password)

    qr = qrcode.QRCode(border=2, box_size=8)
    qr.add_data(payload)
    qr.make(fit=True)
    qr_img = qr.make_image(fill_color="black", back_color="white").convert("RGB")

    margin = 20
    text_width = 320  # generous margin: "Password: " + 16 Crockford base32 chars measures ~256px at font size 16
    label_height = max(qr_img.height, 160) + 2 * margin
    label_width = qr_img.width + text_width + 3 * margin

    label = Image.new("RGB", (label_width, label_height), "white")
    label.paste(qr_img, (margin, margin))

    draw = ImageDraw.Draw(label)
    title_font = load_font(20)
    text_font = load_font(16)

    text_x = qr_img.width + 2 * margin
    text_y = margin
    draw.text((text_x, text_y), "Larus Sensor WiFi", font=title_font, fill="black")
    text_y += 32
    draw.text((text_x, text_y), f"SSID: {ssid}", font=text_font, fill="black")
    text_y += 24
    draw.text((text_x, text_y), f"Password: {password}", font=text_font, fill="black")
    text_y += 24
    # Not part of the QR payload (the WIFI: format has no field for it) -
    # printed as plain text instead so the label also tells the user where
    # to go once they've joined the network.
    draw.text((text_x, text_y), f"Web UI: http://{ip}", font=text_font, fill="black")
    text_y += 32
    draw.text((text_x, text_y), "Scan to connect, or", font=text_font, fill="black")
    text_y += 20
    draw.text((text_x, text_y), "enter manually above.", font=text_font, fill="black")

    return label


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--ssid", help="AP SSID, e.g. as printed by the firmware over serial at boot")
    source.add_argument(
        "--serial-port",
        help="serial port to read 'WiFi AP SSID: ...' / 'WiFi AP password: ...' from directly (e.g. /dev/ttyUSB0, COM3)",
    )
    parser.add_argument("--password", help="AP password (required together with --ssid)")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate for --serial-port (default: 115200)")
    parser.add_argument(
        "--ip",
        default="192.168.4.1",
        help="IP address printed on the label as the web UI address (default: 192.168.4.1, the fixed AP-mode "
        "address from wifi_config.cpp - override if generating a label for a device already in WiFi client mode)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="output PNG path (default: build/larus_wifi_label_<ssid>.png, repository root)",
    )
    args = parser.parse_args()

    if args.ssid and not args.password:
        parser.error("--password is required together with --ssid")

    if args.serial_port:
        ssid, password = read_ssid_password_from_serial(args.serial_port, args.baud)
    else:
        ssid, password = args.ssid, args.password

    print(f"SSID:     {ssid}")
    print(f"Password: {password}")

    output = args.output
    if output is None:
        build_dir = REPO_ROOT_DIR / "build"
        build_dir.mkdir(parents=True, exist_ok=True)
        output = build_dir / f"larus_wifi_label_{ssid}.png"

    label = build_label_image(ssid, password, args.ip)
    label.save(output)
    print(f"Label written to {output}")


if __name__ == "__main__":
    main()
