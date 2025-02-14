![N|Solid](https://arribada.org/wp-content/uploads/2022/01/arribada_web_logo_g.svg)
# Arduino sample code used with Argos SMD wings

## Overview
This repository contains the Arduino sample code designed specifically for testing SPI commands with Argos SMD Wings and the Argos SMD module. The code runs on the Adafruit Feather nRF52840 board and demonstrates SPI communication with the Argos SMD module.

Adapted from the Bluefruit BLE_UART example, this sketch replaces traditional UART AT commands with SPI commands to configure and interact with the Argos SMD module. It supports configuration, status queries, and periodic uplink message transmission, making it suitable for testing and certification purposes.

Code tested with [argos-smd-at-kineis-firmware](https://github.com/arribada/argos-smd-at-kineis-firmware) version v0.6. Code should be compiled with flag COMM = SPI (defined into Makefile)


## Functionality

By default, the program listens for commands via Serial and processes them by forwarding the appropriate SPI commands to the Argos SMD module. 

## SPI Protocol

 * Slave Device Behavior:
    The SMD module operates as an SPI slave device. It continuously listens for incoming bytes from the master. When it receives a byte that corresponds to a known command, the module prepares an appropriate response. This response is transmitted during the next SPI communication initiated by the master.

    Example – Reading the Device ID:
        The master sends 0x08 (command for reading the ID).
        The SMD slave prepares the device ID as the response.
        In the next SPI transaction, the master sends a dummy byte (e.g., 0x00) to trigger the slave to transmit the prepared ID.

 * Write Request Process:
    For write requests, the master first initiates a write request command. In the following SPI transaction, the master sends the value to be written, and the slave processes and stores this new value accordingly.

 * TX Uplink Message:
    For transmitting an uplink message, the master must provide the size of the TX payload along with the command so that the slave knows how many bytes to expect.

 * Timeout Mechanism:
    All commands have a timeout of 10 seconds. If a request is not processed within this time, the slave returns to its normal listening state.

 * Debug Mode Consideration:
    When Debug is enabled on the SMD module, the time between two commands increases (approximately 200 ms during tests). If Debug is disabled, a delay of around 50 ms should be sufficient between commands.

This enhanced description outlines the SPI communication process with the Argos SMD module, detailing the command-response behavior, write operations, and timing considerations.
## Getting Started

    1. Connect your Feather nRF52840 board with the Argos SMD Wings attached.
	2. Open Serial link via an external software (Tera Term for example)
	3. Display help by typing >?; in UART BLE console.
	
Note 1: The write commands don't support parametrable input, only for test purpose, change them directly from the code.
Note 2: The conf need to be reload before to forward an uplink command, enter the following command:
  * >write_radio_conf; // will forward LDK radio configuration
  * >reload_kmac;
  * >uplink;
Note 3: The default CLS credentials are for testing with Emulsat only. Please contact CLS for new credentials if you want to test with a satellite.
	- ID=123456
	- ADDR=11223344
	- SECKEY=00112233445566778899AABBCCDDEEFF

## Commands

Enter commands via Bluetooth in the following format:

    Start with the character '>'
    End with the character ';'

Available commands:
	>?;              - Display the help menu
	>ping;           - Send a ping command to the device
	>spi_status;     - Retrieve the current SPI status
	>mac_status;     - Retrieve the current MAC status
	>spimac_status;  - Retrieve the SPI MAC state
	>version;        - Retrieve the device version
	>firmware;       - Retrieve the firmware version
	>address;        - Read the device address (11223344 written to SMD))
	>write_addr;     - Write a new device address
	>seckey;         - Read the security key
	>write_seckey;   - Write a new security key (00112233445566778899AABBCCDDEEFF written to SMD)
	>id;             - Read the device ID
	>write_id;       - Write a new device ID in flash memory (123456 written to SMD)
	>tcxo;           - Retrieve the TCXO status
	>write_tcxo;     - Write new TCXO settings (3000 written to SMD)
	>sn;             - Read the device serial number
	>read_radio_conf; - Read the current radio configuration
	>write_radio_conf;- Write a new radio configuration (LDK conf written to SMD
	>save_radio_conf; - Save the current radio configuration
	>read_kmac;      - Read the KMAC profile
	>reload_kmac;    - Reload the KMAC configuration
	>read_lpm;       - Read the current Low Power Mode (LPM) (0 written to SMD)
	>write_lpm;      - Write a new Low Power Mode setting
	>uplink;         - Send an uplink message



# Notes
- The following pins have been adapted in the variant.h file located at {ArduinoPath}\Arduino15\packages\adafruit\hardware\nrf52\1.5.0\variants\feather_nrf52840_express\variant.h

```cpp
#define SPI_SCK  (26)    ///< Default SPI Clock (SCK) pin.
#define SPI_MOSI (24)    ///< Default SPI Master Out Slave In (MOSI) pin.
#define SPI_MISO (25)    ///< Default SPI Master In Slave Out (MISO) pin.
#define SPI_CS   (2)     ///< Custom Chip Select (CS) pin.
```

# License

This project is licensed under the GNU General Public License v3. See the LICENSE file for details.
