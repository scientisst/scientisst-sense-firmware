#!/bin/sh

#Get current script's absolute path
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export IDF_PATH="$SCRIPT_PATH/deps/esp-idf/"
export IDF_TOOLS_PATH="$SCRIPT_PATH/esp-idf-tools/"

#If an argument is not supplited (don't skip the export part)
if [ $# -eq 0 ]
    then
    . ./deps/esp-idf/export.sh
fi