# Scientisst - Sense Firmware

The firmware for the Scientisst - Sense development board

---

## Repository structure

```
- deps              :   External Dependencies
- main              :   Firmware Source Files
- sdkconfig         :   ESP32 Hardware Configurations
- www               :   Configuration HTML Page
``` 

## Installing

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

## Flash Firmware (Linux/MacOS)

```sh
. get_idf.sh
#Flash Firmware. Sense must be in flash mode: press MODE (without releasing), press RESET (without releasing), release RESET, release MODE.
idf.py flash
```

## Flash Firmware (Windows)

```sh
get_idf.bat
#Flash Firmware. Sense must be in flash mode: press MODE (without releasing), press RESET (without releasing), release RESET, release MODE.
idf.py flash
```

### Troubleshooting

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

## Serial Monitor (Linux/MacOS)

```sh
idf_monitor
```

To exit the serial monitor, press `CTRL+T` followed by `CTRL+X`

## Licensing

This project is covered by multiple licenses:

- **Apache License, Version 2.0 (Apache-2.0)**: This license applies to certain parts of the project. It is a permissive
  license that allows for free use, modification, and distribution. More details can be found in the [LICENSE](LICENSE)
  file.

- **BSD 3-Clause License (BSD-3-Clause)**: The BSD 3-Clause License covers specific components, particularly those
  related to Bosch Sensortec GmbH. This license also allows for free use, modification, and distribution, with some
  conditions. The full text and specifics are available in the [LICENSE](LICENSE) file.

When using, modifying, or distributing this project, please ensure that you respect and follow the conditions laid out
in both licenses. If you incorporate this project into your work or use it as a dependency, make sure to include the
proper license notices and files.

## Disclaimer

This is not a medical device certified for diagnosis or treatment. It is provided to you as is only for research and
educational purposes.

## Acknowledgments

This work was partially supported by Fundação para a Ciência e Tecnologia (FCT) under the projects’ UIDB/50008/2020 and
PCIF/SSO/0163/2019 (SafeFire), and by the Active Assisted Living Programme Collaborative Project AAL-2020-7-237-CP (
FORTO 2.0) through IT—Instituto de Telecomunicações, which is gratefully acknowledged.
