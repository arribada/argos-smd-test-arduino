# Arduino sample code used with Argos SMD wings

## Overview
This repository contains the Arduino sample code designed specifically for use with Argos SMD FeatherWings and the Argos SMD module. It is deployed on the nRF52840 Feather board from Adafruit and serves as a robust implementation for interacting with the Argos SMD module.

The code is derived from the Bluefruit BLE_UART example, tailored to seamlessly integrate with the Argos SMD hardware. It demonstrates key functionalities such as configuration, communication, and power management for the Argos module.

This code was successfully used during the certification process with CLS (Collecte Localisation Satellites), ensuring compliance with Argos standards and demonstrating the reliability of the solution. Its compatibility and performance were rigorously tested to meet certification requirements.

Code tested with [argos-smd-at-kineis-firmware](https://github.com/arribada/argos-smd-at-kineis-firmware) version v0.4.

## Hardware Requirements

- Argos SMD Wings (v1.0) + argos SMD module v1.0 (for Argos communication via Serial)
- Feather nRF52840 (main scheduler, Bluetooth, LED control, and Serial communication with Argos SMD Wings and Wings_GNSS)

## Functionality

By default, the program forwards the same LDA2 message every 50 seconds with a jitter of ±10% (uniformly distributed between 45 and 55 seconds).

## Getting Started

    1. Download the Bluefruit Connect app and connect to your device.

    2. Blue LED states:
        Blinking: Waiting for Bluetooth connection
        Fixed: Connected

    3. Go to the UART tab in the app to see the device trace and enter commands.

## Commands

Enter commands via Bluetooth in the following format:

    Start with the character '>'
    End with the character ';'

Available commands:

    >?; - Display the help menu
    >test_mode; - Enable/disable periodic SAT message (default ON)
    >pulse; - Send Argos test message
    >msg=Message; - Change the message sent via Argos
    >set_period=Value; - Set transmission period in milliseconds
    >set_jitter=Value; - Set jitter percentage for transmissions
    >conf; - Read SMD configuration
    >conf_LDA2; - Set SMD radio to LDA2 mode
    >conf_LDA2L; - Set SMD radio to LDA2L mode
    >conf_VLDA4; - Set SMD radio to VLDA4 mode
    >conf_LDK; - Set SMD radio to LDK mode
    >conf_reload; - Reload the KMAC profile
    >ID; - Read SMD ID
    >SN; - Read SMD Serial Number
    >ADDR; - Read SMD Address
    >FW; - Read SMD firmware version
    >AT_VERSION; - Read AT version from SMD
    >ping; - Ping SMD module
    >read_LPM; - Read SMD Low Power Mode
    >udate; - Read UTC date from SMD
    >CW=Mode,Freq,Power; - Set continuous wave mode
    >LPM=Mode; - Set Low Power Mode

# Notes

- The USB virtual Serial (Serial) is not used in this example. If you want to replace the BLE app and use only USB communication, replace the traceOutput variable from bleuart to Serial. You also need to uncomment the Serial initialization in the setup() function.

- The following pins have been adapted in the variant.h file located at {ArduinoPath}\Arduino15\packages\adafruit\hardware\nrf52\1.5.0\variants\feather_nrf52840_express\variant.h

```cpp
    #define PIN_SERIAL1_RX       (12)
    #define PIN_SERIAL1_TX       (13)
    #define PIN_SERIAL2_RX       (1)
    #define PIN_SERIAL2_TX       (0)
```
If you don't want to change the variant.h file, please edit the Serial2 pinout.

# License

This project is licensed under the GNU General Public License v3. See the LICENSE file for details.