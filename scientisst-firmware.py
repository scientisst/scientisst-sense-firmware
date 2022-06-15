import argparse
import os
from pathlib import Path
import argparse
import platform
import sys
import subprocess

#ABS_PATH = str(Path().absolute())

parser = argparse.ArgumentParser(description='ScientISST-SENSE Firmware Tool')
group = parser.add_mutually_exclusive_group(required=True)  #Make these args mutually exclusive, i.e. only one arg is allowed

group.add_argument(
        "-i",
        "--install",
        dest="install",
        action="store_true",
        default=False,
        help="Install esp-idf tools in order to build firmware",
    )

group.add_argument(
        "-b",
        "--build",
        dest="build",
        action="store_true",
        default=False,
        help="Build firmware",
    )

group.add_argument(
        "-c",
        "--clean",
        dest="clean",
        action="store_true",
        default=False,
        help="Clean build outputs",
    )

group.add_argument(
        "-f",
        "--flash",
        dest="flash",
        action="store_true",
        default=False,
        help="Flash firmware",
    )

group.add_argument(
        "-v",
        "--version",
        dest="version",
        action="store_true",
        default=False,
        help="Show firmware version",
    )

group.add_argument(
        "-m",
        "--monitor",
        dest="monitor",
        default=None,
        nargs='?',
        const='/dev/ttyUSB0',
        help="Open serial monitor in <MONITOR> port. Example: scientisst-firmware.py --monitor /dev/ttyUSB0",
    )

args = parser.parse_args()

SCRIPT_EXT = '.bat' if platform.system() == "Windows" else '.sh'
GET_IDF_PATH = './get_idf' + SCRIPT_EXT

SOURCE_GET_IDF = '. ' + GET_IDF_PATH
if args.install:
    cmd = SOURCE_GET_IDF + " --no_export" + " && " + "$IDF_PATH/" + 'install' + SCRIPT_EXT + " esp32"
else:
    if args.build:
        cmd = SOURCE_GET_IDF+ ' && idf.py build'
    elif args.clean:
        cmd = SOURCE_GET_IDF + ' && idf.py fullclean'
    elif args.flash:
        cmd = SOURCE_GET_IDF + ' && idf.py flash'
    elif args.version:
        print("Not yet implemented")
    elif args.monitor:
        cmd = SOURCE_GET_IDF + ' && idf.py monitor' + args.monitor

os.system('/bin/sh ' + cmd)