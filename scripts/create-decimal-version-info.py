#!/bin/python

import subprocess
import re

version_string = subprocess.check_output("git describe --always --dirty --tags", shell=True).decode('utf-8')
version = version_string.split('.')
match = re.match('(?P<first>[0-9]*).(?P<second>[0-9]*).(?P<third>[0-9]*)-(?P<build>[0-9]*)-.*',version_string)

first = int(match.group('first'))
second = int(match.group('second'))
third = int(match.group('third'))
build = int(match.group('build'))

print('Version {}.{}.{} Build {}'.format(first,second,third,build))
# TODO: convert groups to hex and write C / C++ define line to git-commit-version.h
# e.g. "#define VERSION_TXT6 0x%02x%02x%04x\r\n" $Major $Minor $Micro >> $VersionFile



