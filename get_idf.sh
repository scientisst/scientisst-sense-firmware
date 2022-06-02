#!/bin/sh

#Get current script's absolute path
SCRIPTPATH="$( cd -- "$( dirname -- "${BASH_SOURCE[0]:-$0}"; )" &> /dev/null && pwd 2> /dev/null; )";

export IDF_TOOLS_PATH="$SCRIPTPATH/esp-idf-tools/"
. ./deps/esp-idf/export.sh