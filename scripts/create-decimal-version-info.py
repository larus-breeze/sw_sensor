#!/bin/python

import subprocess
import re

version_string = subprocess.check_output("git describe --always --dirty --tags", shell=True).decode('utf-8')
match = re.match('(?P<first>[0-9]*).(?P<second>[0-9]*).(?P<third>[0-9]*)-(?P<build>[0-9]*)-.*',version_string)

first = int(match.group('first'))
second = int(match.group('second'))
third = int(match.group('third'))
build = int(match.group('build'))

defstring = '#define GIT_TAG_DEC 0x{:02x}{:02x}{:02x}{:02x}'.format(first,second,third,build)

with open("Core/Inc/git-commit-version.h", "a") as file:
    file.write('\n') 
    file.write(defstring)



