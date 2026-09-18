#!/bin/python3

import sys, io, os, time, toml, struct

from elftools.elf.elffile import ELFFile
from elftools.elf.relocation import RelocationSection

# Resolve every path against this script's own location (sw_stm32/scripts),
# not the current working directory, so it runs the same whether invoked
# from sw_sensor/ or from sw_sensor/sw_stm32/.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.dirname(SCRIPT_DIR)  # sw_stm32/
REPO_ROOT_DIR = os.path.dirname(BASE_DIR)  # sw_sensor/
# Final packaged images land in a shared top-level build/ folder (gitignored),
# alongside the ESP32 side's own build_firmware.py output, instead of
# sw_stm32/ - so both firmware images end up next to each other.
BUILD_DIR = os.path.join(REPO_ROOT_DIR, "build")

def warn_if_stale(elf_path, max_age_seconds=600):
    """Return a warning message if elf_path is older than max_age_seconds -
    a common sign that the wrong build configuration (e.g. Debug instead of
    Release) was active last, leaving a stale image sitting in the Release
    folder. Returns None if the file is fresh enough."""
    age_seconds = time.time() - os.path.getmtime(elf_path)
    if age_seconds > max_age_seconds:
        return (f"WARNING: '{elf_path}' is {age_seconds / 60:.1f} minutes old - "
                f"make sure it was actually just built with the Release "
                f"configuration, not an older or Debug build.")
    return None


def stm32_crc(data):
    crc=0xffffffff
    buf = bytearray()
    for b in data:
        buf.insert(0, b)
        if len(buf) == 4:
            for val in buf:
                crc ^= val << 24
                for _ in range(8):
                    crc = crc << 1 if (crc & 0x80000000) == 0 else (crc << 1) ^ 0x104c11db7
            buf = bytearray()
    return crc

def get_version(version_str):
        """Packs a dotted version string (e.g. "0.7.6.193") into a 32-bit int,
        one byte per component - so each component must fit in 0-255. The 4th
        component (build/commit number) is the one that keeps growing and
        will eventually hit this on its own, silently corrupting the packed
        version (or making struct.pack below raise) if not caught here."""
        shift_fact = 0
        version = 0
        for no in version_str.split('.'):
            value = int(no)
            if not (0 <= value <= 255):
                sys.exit(
                    f"version component '{no}' in '{version_str}' is {value}, "
                    f"outside the 0-255 range this packed format allows (1 byte "
                    f"per component). If this is the build/commit-number "
                    f"component, the packed version format needs widening "
                    f"before this can keep growing."
                )
            version += value << shift_fact
            shift_fact += 8

        return version

class ReadApp():
    """Read elf file and store all binaries and symbols"""
    def __init__(self, file_name):
        in_stream =  open(file_name, "rb")
        self.elf_file = ELFFile(in_stream)
        self.file_name = file_name
    def get_binary(self, flash_start, flash_end):
        """Return the binary data that lies within the defined range"""
        last_adr = 0
        segment_data = []

        print(f"\nLoading app image from segments in '{self.file_name}'")
        print(f"    {'Address':8}   {'Length':8}")
        for segment in self.elf_file.iter_segments():
            addr = segment['p_paddr']
            if addr >= flash_start and addr < flash_end:
                data = segment.data()
                length = len(data)
        
                print(f"  0x{addr:08X} 0x{length:08X}")
                segment_data.append((addr - flash_start, data))
                last_adr = addr + length

        binary_len = last_adr - flash_start
        binary = bytearray(binary_len)
        for ofs, data in segment_data:
            binary[ofs:ofs + len(data)] = data

        # Be shure, that binary is word aligned
        while len(binary) % 4 != 0: # Go safe to get 4 byte alignment 
            binary += b'\x00'

        self.binary = binary
        return binary

    def get_symbol_address(self, symbol_name):
        """Return the address of the symbol"""
        symtab = self.elf_file.get_section_by_name('.symtab')
        symbol = symtab.get_symbol_by_name(symbol_name)
        return symbol[0].entry['st_value']

class Binary():
    """A class to create binary Larus images"""
    def __init__(self, image):
        """storage_adr: Address where the image is to be stored"""
        self.name = image["name"]
        self.addr_storage = image["addr_storage"]
        self.hw_version = get_version(image["hw_version"])
        self.sw_version = get_version(image["sw_version"])
        self.stale_warning = None

    def read_new_app(self, app):
        """Load the app that is to be executed later"""
        elf_path = os.path.join(BASE_DIR, app["elf"])
        self.stale_warning = warn_if_stale(elf_path)
        new_app = ReadApp(elf_path)
        self.app_addr_start = app["addr_start"]
        app_addr_max = app["addr_max"]
        self.app_bin = new_app.get_binary(self.app_addr_start, app_addr_max)

    def read_copy_app(self, copy):
        """Load the copy routine that loads the future app in the right place."""
        copy_app = ReadApp(os.path.join(BASE_DIR, copy["elf"]))
        self.copy_app_addr_start = copy["addr_start"]
        copy_app_addr_max = copy["addr_max"]
        self.copy_bin = copy_app.get_binary(self.copy_app_addr_start, copy_app_addr_max)
        self.copy_func = copy_app.get_symbol_address("main")

    def create_meta_data(self):
        """Create the meta data needed"""
        data = {
             'Magic Number': 0x1c80_73ab_2085_3579,
             'CRC <place holder>': 0x12345678,
             'Meta Data Version': 1,
             'Storage Address': self.addr_storage,
             'Hardware Version': self.hw_version,
             'Software Version': self.sw_version,
             'Copy Function': self.copy_func,
             'New App': self.addr_storage + 0x1000 + len(self.copy_bin),
             'New App Len': len(self.app_bin),
             'New App Dest': self.app_addr_start
        }
        print('\nCreating Meta Data:')
        for key, value in data.items():
            print(f"  {key:21}0x{value:08X}")


        self.meta_data = struct.pack ('<QLLLLLLLLL', *data.values())
        while len(self.meta_data) < (self.copy_app_addr_start - self.addr_storage): # Fill til copy 
            self.meta_data += b'\x00'

    def write_file(self):
        """Save binary to disk"""
        self.create_meta_data()
        binary = bytearray(self.meta_data + self.copy_bin + self.app_bin)

        print("\nCalculate CRC 'Storage Address' -> <end>")
        crc_data = stm32_crc(binary[12:]) # Start at storag_adr -> end
        binary[8:12] = struct.pack("<L", crc_data)
        print(f"  CRC inserted         0x{crc_data:08X}")

        print(f"\nTotal size of binary: {round(len(binary) / 1024)}k")
        os.makedirs(BUILD_DIR, exist_ok=True)
        out_path = os.path.join(BUILD_DIR, self.name)
        print(f"Writing binary to file '{out_path}'")
        with open(out_path, "wb") as bin_file:
            bin_file.write(binary)

        if self.stale_warning:
            print(f"\n{self.stale_warning}")

if ((len(sys.argv) == 2) and ("LEGACY" in str(sys.argv[1]))):
    print("Larus App Image Packer - Legacy")
    with open(os.path.join(SCRIPT_DIR, "pack_legacy.toml"), "r") as f:
        spec = toml.load(f)
        image = Binary(spec["image"])
        image.read_new_app(spec["app"])
        image.read_copy_app(spec["copy"])
        image.write_file()

else:
    print("Larus App Image Packer")
    with open(os.path.join(SCRIPT_DIR, "pack.toml"), "r") as f:
        spec = toml.load(f)
        image = Binary(spec["image"])
        image.read_new_app(spec["app"])
        image.read_copy_app(spec["copy"])
        image.write_file()



