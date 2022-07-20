#!/bin/sh

#Get current script's absolute path
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export IDF_PATH="$SCRIPT_PATH/deps/esp-idf/"
export IDF_TOOLS_PATH="$SCRIPT_PATH/esp-idf-tools/"

#If an argument is not supplited (don't skip the export part)
if [ $# -eq 0 ]; then
    #Export idf
    . $IDF_PATH/export.sh

    #Create command to open serial monitor with ease
    alias idf_monitor="idf.py monitor -p /dev/ttyUSB0"

#Check for arguments
else
    POSITIONAL_ARGS=()
    while [[ $# -gt 0 ]]; do
        case $1 in
            --install)
            INSTALL=1
            shift # past argument
            ;;
            -*|--*)
            echo "Unknown option $1"
            exit 1
            ;;
            *)
            POSITIONAL_ARGS+=("$1") # save positional arg
            shift # past argument
            ;;
        esac
    done

    set -- "${POSITIONAL_ARGS[@]}" # restore positional parameters

    if [ "$INSTALL" -eq "1" ]; then
        ${SCRIPT_PATH}/deps/esp-idf/install.sh
    fi
fi