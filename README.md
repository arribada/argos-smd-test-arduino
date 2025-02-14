![N|Solid](https://arribada.org/wp-content/uploads/2022/01/arribada_web_logo_g.svg)
# Arduino sample code used with Argos SMD wings

## Overview
This repository contains sample Arduino code for testing the Argos SMD module mounted on the Argos SMD Wings using a Feather board. Two distinct code bases are provided:
 * argos-smd-certfication-sample: Test code based on BLE and UART AT commands manager.
 * argos-smd-spi-wrapper: Test code based on Serial and SPI commands manager.

Both examples demonstrate key functionalities such as configuration, communication, and power management for the Argos SMD module. The SPI-based code, in particular, showcases robust SPI communication for sending commands and receiving responses from the module.

## Hardware Requirements

* Argos SMD Wings (v1.0) combined with an Argos SMD module for SPI communication.
* Feather nRF52840 (serving as the main scheduler, handling Bluetooth connectivity, LED control, and SPI communication with the Argos SMD Wings).


# License

This project is licensed under the GNU General Public License v3. See the LICENSE file for details.
