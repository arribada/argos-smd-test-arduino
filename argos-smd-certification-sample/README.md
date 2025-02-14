![N|Solid](https://arribada.org/wp-content/uploads/2022/01/arribada_web_logo_g.svg)
# Arduino sample code used with Argos SMD wings

## Overview
This repository contains the Arduino sample code designed specifically for use with Argos SMD FeatherWings and the Argos SMD module. It is deployed on the nRF52840 Feather board from Adafruit and serves as a robust implementation for interacting with the Argos SMD module.

The code is derived from the Bluefruit BLE_UART example, tailored to seamlessly integrate with the Argos SMD hardware. It demonstrates key functionalities such as configuration, communication, and power management for the Argos module.

This code was successfully used during the certification process with CLS (Collecte Localisation Satellites), ensuring compliance with Argos standards and demonstrating the reliability of the solution. Its compatibility and performance were rigorously tested to meet certification requirements.

Code tested with [argos-smd-at-kineis-firmware](https://github.com/arribada/argos-smd-at-kineis-firmware) version v0.4.

## Functionality

By default, the program listens for commands via Serial (which can be sent via Bluetooth using the Bluefruit Connect app) and processes them by forwarding the appropriate SPI commands to the Argos SMD module. In addition, it periodically sends an uplink message every 50 seconds with a jitter of ±10% (i.e., a random delay between 45 and 55 seconds).

## Getting Started

    1. Connect your Feather nRF52840 board with the Argos SMD Wings attached.
	2. Download the [Bluefruit Connect app](https://learn.adafruit.com/bluefruit-le-connect/ios-setup) and connect to your device.
    2. Blue LED states:
        Blinking: Waiting for Bluetooth connection
        Fixed: Connected

    3. Open the Serial/UART tab in the Bluefruit Connect app to view the device trace and enter commands.
	4. Reset the SMD module to see initialization log and working UART.
	5. Display help by typing >?; in UART BLE console.
	
Note 1: If you don't see the Init log of the SMD module, try to reset the Feather board, wait the automatic bluetooth connection and reset again the SMD module.
Note 2: If no uplink message is forwarded, try to reload the radio_conf with (`>conf_XXX;` command followed by `>conf_reload`)
Note 3: The default CLS credentials are for testing with Emulsat only. Please contact CLS for new credentials if you want to test with a satellite.
	- ID=123456
	- ADDR=11223344
	- SECKEY=00112233445566778899AABBCCDDEEFF

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
	>ID=XXXXXX; Write SMD ID in flash memory
    >SN; - Read SMD Serial Number
    >ADDR; - Read SMD Address
	>ADDR=XXXXXXXX; - Write HEX address in flash memory
	>SECKEY; - Read Secret key
	>SECKEY=XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX; - Write Secrete key in flash memory
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
