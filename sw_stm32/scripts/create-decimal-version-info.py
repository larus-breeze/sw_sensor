#!/bin/python3
import subprocess
import re

version_string = subprocess.check_output("git describe --always --dirty --tags", shell=True).decode('utf-8')

first = 0xff
second = 0xff
third = 0xff
build = 0xff

try:
    # Try to match tag and build number
    match = re.match('(?P<first>[0-9]*).(?P<second>[0-9]*).(?P<third>[0-9]*)-(?P<build>[0-9]*)-.*', version_string)
    first = int(match.group('first'))
    second = int(match.group('second'))
    third = int(match.group('third'))
    build = int(match.group('build'))

except Exception as e:
    # Try to match tag only from e newly created version.
    try:
        match = re.match('(?P<first>[0-9]*).(?P<second>[0-9]*).(?P<third>[0-9]*).*', version_string)
        first = int(match.group('first'))
        second = int(match.group('second'))
        third = int(match.group('third'))
        build = 0

    except Exception as e:
        print("Something went wrong getting the TAG version information!: ", e)

defstring = '#define GIT_TAG_DEC 0x{:02x}{:02x}{:02x}{:02x}'.format(first,second,third,build)
print(defstring)

with open("Core/Inc/git-commit-version.h", "a") as file:
    file.write('\n')
    file.write(defstring)
