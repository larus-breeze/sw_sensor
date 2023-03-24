#!/bin/sh

VersionFile="Core/Inc/git-commit-version.h"
  read loc_string < $VersionFile

if [ -r $TagFile ]; then
  # tokenizing
  read loc_string < $VersionFile
  # create numbers from sring tokens
  OLDIFS="$IFS"
  IFS='.' tokens=( $loc_string )
  echo "set Major="${tokens[0]}
  Major=${tokens[0]}
  echo "set Minor="${tokens[1]}
  Minor=${tokens[1]}
  echo "set Micro="${tokens[2]}
  Micro=${tokens[2]}
  IFS="$OLDIFS" # restore IFS

else
#  No file found set version to 0
   Major=0
   Minor=0
   Micro=0
fi

# VersionFile="Core/Inc/git-commit-version.h" extended by one line
# Hinweis: Zwischen dem abschliessenden " und \ muss ein Blank stehen !!!
#                                               ====

echo "#define VERSION_TXT6 0x%02x%02x%04x\r\n" $Major $Minor $Micro >> $VersionFile

