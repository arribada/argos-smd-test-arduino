# HorizonLinks FeatherWings + Argos SMD Module Example

This example demonstrates how to use the HorizonLinks FeatherWings with the Argos SMD module connected to an nRF52840 Feather board. It is based on the Bluefruit app BLE_UART example.
## Hardware Requirements

- HorizonLink FeatherWings + STM32WL SMD module (for Arctic communication via Serial)
- Feather nRF52840 (main scheduler, Bluetooth, LED control, and Serial communication with Wings_Breakout and Wings_GNSS)

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

            >?; => Display the help menu
            >test_mode; => Enable/disable periodic SAT message (default ON)
            >pulse; => Send Argos test message
            >msg=Message; => Change the message sent via Argos
            >set_period=Value; => Set transmission period in milliseconds
            >set_jitter=Value; => Set jitter percentage for transmissions
            >conf; => Read SMD configuration
            >conf_LDA2; => Set SMD radio to LDA2 mode
            >conf_LDA2L; => Set SMD radio to LDA2L mode
            >conf_VLDA4; => Set SMD radio to VLDA4 mode
            >conf_LDK; => Set SMD radio to LDK mode
            >conf_SAVE; => Save radio conf
            >conf_RELOAD; => Reload the new radio configuration without reset required
            >ID; => Read SMD ID
            >SN; => Read SMD Serial Number
            >ADDR; => Read SMD Address
            >FW; => Read SMD firmware version
            >AT_VERSION; => Read AT version from SMD
            >ping; => Ping SMD module
            >read_LPM; => Read SMD Low Power Mode
            >udate; => Read UTC date from SMD
            >CW=Mode,Freq,Power; => Set continuous wave mode
            >LPM=Mode; => Set Low Power Mode

# Notes

- The USB virtual Serial (Serial) is not used in this example. If you want to replace the BLE app and use only USB communication, replace the traceOutput variable from bleuart to Serial. You also need to uncomment the Serial initialization in the setup() function.

- The following pins have been adapted in the variant.h file located at {ArduinoPath}\Arduino15\packages\adafruit\hardware\nrf52\1.5.0\variants\feather_nrf52840_express\variant.h

```cpp
    #define PIN_SERIAL1_RX       (12)
    #define PIN_SERIAL1_TX       (13)
    #define PIN_SERIAL2_RX       (1)
    #define PIN_SERIAL2_TX       (0)
```

# License

MIT license, check LICENSE for more information. All text above, and the splash screen below, must be included in any redistribution.
