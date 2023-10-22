# Scientisst - Sense Firmware

---

The firmware for the Scientisst - Sense development board

## Table of contents

---

- [General overview](#General overview)
- [Repository structure](#repository-structure)
- [Installing](#installing)
    - [1) Prerequisites](#1-prerequisites)
    - [2) Getting this repository](#2-getting-this-repository)
    - [3) Install Xtensa's toolchain](#3-install-xtensas-toolchain)
    - [4) Load Xtensa's tools](#4-load-xtensas-tools)
- [Configure firmware](#configure-firmware)
- [Flash firmware](#flash-firmware)
- [Troubleshooting](#troubleshooting)
- [Serial monitor](#serial-monitor-linuxmacos)
- [Licensing](#licensing)
- [Disclaimer](#disclaimer)
- [Acknowledgments](#acknowledgments)

## General overview

---

The Scientisst - Sense firmware is based on the ESP-IDF framework, which is Espressif Systems' official development.
This firmware is flashed onto the Scientisst - Sense board (Core, Cardio and Nano) designed in-house, which are based on
the ESP32 microcontroller. The firmware is responsible for initializing the device, configuring the sensors, and
communicating with the various APIs.

The firmware is divided into tasks, each task is responsible for a specific function. The tasks are:

- **main task (sci_scientisst)**: Responsible for initializing the device, configuring the sensors, and creating the
  other tasks. Terminates itself after the setup is complete.
- **sci_task_acquisition**: Responsible for acquiring data from the sensors and storing it in the buffers. A timer is
  used to wake this task up at the desired sampling rate. The sampling rate can be changed at runtime using the API.
- **sci_task_battery_monitor**: Responsible for monitoring the battery level and turning on a red LED when the battery
  is low. Does not interact with other tasks. Only run when not using a Wi-Fi communication mode because ADC2 and Wi-Fi
  cannot be used at the same time.
- **sci_task_com_tx**: Responsible for transmitting the data stored in the buffers to the API.
- **sci_task_com_rx**: Responsible for receiving and processing commands from the API like start/stop acquisition,
  change sampling rate, change API mode, active channels, etc.
- **sci_task_acquisition_sdcard**: When in SD Card mode, responsible for acquiring data from the sensors and storing it
  in the SD Card. This task is created instead of sci_task_acquisition, sci_task_com_tx and sci_task_com_rx.
- **sci_task_imu**: Responsible for acquiring data from the BNO055 IMU at a constant 100 Hz and storing it in static
  variables. Acquisition tasks can use getImuValues() to get the latest data from the IMU.

The firmware supports the following communication modes:

- **Bluetooth Classic**: The device acts as a Bluetooth Classic SPP server and can be connected to a Bluetooth Classic
  SPP client.
- **Bluetooth Low Energy (BLE)**: The device acts as a BLE peripheral and can be connected to a BLE central device. The
  data is transmitted using the BLE GATT protocol.
- **TCP - AP**: The device acts as a TCP server and can be connected to a TCP client.
- **TCP - STA**: The device acts as a TCP client and can be connected to a TCP server.
- **UDP - STA**: The device acts as a UDP client and can be connected to a UDP server.
- **Serial**: The device acts as a serial device and can be connected via USB.
- **SD Card**: The device stores the data in the SD Card. The data can be retrieved by removing the SD Card and using
  the [SDCardFileConveter.py](SDCardFileConverter/sdcardfileconversion.py) script to convert the binary files to CSV
  files. Check [SDCardBinaryFileFormat.md](docs/SDCardBinaryFileFormat.md) for more information about the binary file
  format.
- **Web Socket - AP**: The device acts as a Web Socket server and can be
  connected to a Web Socket client. :warning: **Warning**: Currently not working.
-

## Repository structure

---

```
scientisst-sense-firmware/
├── deps/                           : External Dependencies
│    ├── BNO055_driver/             : Bosch Sensortec's BNO055 driver
│    ├── esp-idf/                   : Espressif Systems' ESP-IDF v4.4.4
├── docs/                           : Documentation of the repository
│    ├── doxygen/                   : Doxygen generated documentation of the main/ directory
│    ├── How_to_flash_scientisst/   : Instructions on how to flash a ScientISST device
│    ├── frames.jpg/.svg            : Layout of the frames used to communicate between the device and the various APIs
├── main/                           : Firmware source files: all ScientISST files follow the format "sci_XXXXX.c/.h"
│    ├── certs/                     : Cryptographic files used for secure communications protocols
│    ├── communication/             : Communication protocols
│    ├── drivers/                   : GPIO, wifi, UART, timer
│    ├── sensors/                   : Internal and external ADCs, BNO055
│    ├── tasks/                     : Tasks created by the firmware
│    ├── CMakeLists.txt             : CMake file
│    ├── component.mk
│    ├── doxygen_config_file        : Doxygen configuration file
│    ├── Kconfig                    : Allows configuring the firmware using the menuconfig tool
│    ├── main.c 
│    ├── sci_macros.h               : Global macros used by the firmware
│    ├── sci_scientisst.c           : Initialization of the device, task selection and creation, buffer allocation.
│    ├── sci_scientisst.h           : Contains global variables
│    └── sci_version.h              : Contains the version of the firmware, automatically updated with each commit
├── SDCardFileConverter/            : Python script to convert the binary files stored in the SD Card to CSV files
├── www/                            : Configuration HTML page
├── CMakeLists.txt                  : CMake file
├── dependencies.lock               : ESP-IDF dependencies lock file
├── get_idf.bat                     : Script to install and export to path the ESP-IDF for Windows
├── get_idf.sh                      : Script to install and export to path the ESP-IDF for Linux/MacOS
├── LICENSE
├── Makefile
├── partitions.csv                  : Custom ESP32 partitions
├── README.md
├── sdkconfig                       : ESP32 ESP-IDF configurations
├── sdkconfig.defaults              : ESP32 ESP-IDF configurations (defaults)
└── update_version.sh               : Script to update the version of the firmware, used as a pre-commit hook to update sci_version.h with the latest commit hash
``` 

## Installing

---

This is an overview of the steps required to install the firmware. For more detailed instructions, please refer to
the [How to flash a ScientISST device](docs/How_to_flash_scientisst/how_to_flash_a_scientisst.ipynb) document.

### 1) Prerequisites

Install the ESP-IDF prerequisites following espressif's
guide https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/#step-1-install-prerequisites

### 2) Getting this repository

```sh
# Getting this repository 
git clone --recursive git@github.com:scientisst/scientisst-sense-firmware.git
```

### 3) Install Xtensa's toolchain

#### (Linux/MacOS)

```sh
. get_idf.sh --install
```

#### (Windows)

```sh
get_idf.bat --install
```

### 4) Load Xtensa's tools

This step must be done every time you open a new terminal.

#### (Linux/MacOS)

```sh
. get_idf.sh 
```

#### (Windows)

```sh
get_idf.bat 
```

## Configure firmware

---

First open the menuconfig tool:

```sh
idf.py menuconfig
```

Then navigate to ScientISST Configuration: Component config -> SCIENTISST Configuration (last option) and configure the
desired options.
The tool does not verify if the physical hardware supports the selected options, so be careful when selecting the
options. Using the monitor tool, you can check if the device is working as expected and if any errors occur.

## Flash firmware

---

### Flash firmware (Linux/MacOS)

```sh
. get_idf.sh
#Flash Firmware. Sense must be in flash mode: press MODE (without releasing), press RESET (without releasing), release RESET, release MODE.
idf.py flash
```

### Flash firmware (Windows)

```sh
get_idf.bat
#Flash Firmware. Sense must be in flash mode: press MODE (without releasing), press RESET (without releasing), release RESET, release MODE.
idf.py flash
```

## Troubleshooting

---

#### Cannot open /dev/ttyUSB0: Permission denied

This is not an issue with the ScientISST SENSE, and its not actually a bug at all, its just part of Linux. Its best
practice to not change permissions in /dev unless as a last resort. What you want to do instead is to add yourself to
the group which would give you permission to access the tty ports.

To see the groups you are in simply type:

```sh
groups
```

To see all available groups type:

```sh
compgen -g
```

Most of them are self-explanatory, in this case you want to add yourself to either the tty group and dialout group,
which you would do by:

```sh
sudo usermod -a -G tty <your username>
sudo usermod -a -G dialout <your username>
```

Then your user should have access to tty without use of sudo.

## Serial monitor (Linux/MacOS only)

---

```sh
idf_monitor
```

To exit the serial monitor, press `CTRL+T` followed by `CTRL+X`

## Licensing

---

This project is covered by multiple licenses. The full text is available in the [LICENSE](LICENSE) file.

- **Apache License, Version 2.0 (Apache-2.0)**: This license applies to certain parts of the project. It is a permissive
  license that allows for free use, modification, and distribution.

- **BSD 3-Clause License (BSD-3-Clause)**: The BSD 3-Clause License covers specific components, particularly those
  related to Bosch Sensortec GmbH. This license also allows for free use, modification, and distribution, with some
  conditions.

When using, modifying, or distributing this project, please ensure that you respect and follow the conditions laid out
in both licenses. If you incorporate this project into your work or use it as a dependency, make sure to include the
proper license notices and files.

## Disclaimer

---

This is not a medical device certified for diagnosis or treatment. It is provided to you as is and only for research and
educational purposes.

## Acknowledgments

---

This work was partially supported by Fundação para a Ciência e Tecnologia (FCT) under the projects’ UIDB/50008/2020 and
PCIF/SSO/0163/2019 (SafeFire), and by the Active Assisted Living Programme Collaborative Project AAL-2020-7-237-CP (
FORTO 2.0) through IT—Instituto de Telecomunicações, which is gratefully acknowledged.
