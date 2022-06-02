import argparse
import os
from pathlib import Path
import argparse
import platform
import sys

ABS_PATH = str(Path().absolute())

ESP_IDF_PATH = ABS_PATH + '/deps/esp-idf/'

SCRIPT_EXT = '.bat' if platform.system() == "Windows" else '.sh'
EXPORT_PATH = ESP_IDF_PATH + 'export' + SCRIPT_EXT
INSTALL_PATH = ESP_IDF_PATH + 'install' + SCRIPT_EXT
GET_IDF_PATH = './get_idf' + SCRIPT_EXT

GET_IDF = ('call ' if platform.system() == "Windows" else '. ') + GET_IDF_PATH


parser = argparse.ArgumentParser(description='ScientISST Firmware Tool')
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

if args.install:
    IDF_TOOLS_PATH_PATH = ABS_PATH + "/esp-idf-tools/"
    os.environ["IDF_TOOLS_PATH"] = IDF_TOOLS_PATH_PATH
    os.system(INSTALL_PATH)
else:
    if args.build:
        os.system(GET_IDF + ' && idf.py build')
    elif args.clean:
        os.system(GET_IDF + ' && idf.py fullclean')
    elif args.flash:
        os.system(GET_IDF + ' && idf.py flash')
    elif args.version:
        print("Not yet implemented")
    elif args.monitor:
        os.system(GET_IDF + ' && idf.py monitor' + args.monitor)