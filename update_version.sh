#!/bin/bash

VERSION_FILE="./main/sci_version.h"

# get the latest commit hash
commit_hash=$(git rev-parse --short HEAD)

# get the current firmware version from the VERSION_FILE
firmware_version=$(grep -oP '(?<=FIRMWARE_VERSION \").*(?=\")' $VERSION_FILE)

# extract the major, minor, and patch version from the firmware version
major_version=$(echo $firmware_version | awk -F "." '{print $1}')
minor_version=$(echo $firmware_version | awk -F "." '{print $2}')
patch_version=$(echo $firmware_version | awk -F "." '{print $3}' | awk -F "-" '{print $1}')

# get the increment flag from the VERSION_FILE
increment_flag=$(grep -oP '(?<=VERSION_CAN_INCREMENT_FLAG )[0-1]' $VERSION_FILE)

# check if the increment flag is 1
if [ "$increment_flag" == "1" ]; then
  # increment the minor version
  minor_version=$((minor_version + 1))

  # replace the minor version in the file
  sed -i "s/MINOR_VERSION \".*/MINOR_VERSION \"$minor_version\"/" $VERSION_FILE
else
  # set the minor version to 0 if the increment flag is 0
  minor_version=0
fi

# update the firmware version with the latest commit hash and new minor version
sed -i "s/FIRMWARE_VERSION \".*\"/FIRMWARE_VERSION \"$major_version.$minor_version.$patch_version-$commit_hash\"/" $VERSION_FILE

git add main/sci_version.h
