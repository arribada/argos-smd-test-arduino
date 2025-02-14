/**********************************************************************
 * @file    argos-smd-spi-wrapper.ino
 * @brief   Test SPI Commands of Argos Module via SMD Wings.
 *
 * This Arduino sketch is designed to test SPI command communication with the
 * Argos SMD module using HorizonLinks FeatherWings. It is based on the Bluefruit
 * Connect BLE_UART example and runs on hardware such as the Feather nRF52840.
 *
 * The sketch initializes SPI communication, configures a NeoPixel LED for visual
 * feedback, and uses Serial for debug output and command input. Commands are
 * expected to be entered using a specific format:
 *    - Each command starts with the character '>'
 *    - Each command ends with the character ';'
 *
 * The available commands and their functions are as follows:
 *    >?;               - Display the help message with the list of available commands.
 *    >ping;            - Send a ping command to the Argos module to check connectivity.
 *    >spi_status;      - Retrieve and display the current status of the SPI interface.
 *    >mac_status;      - Retrieve and display the current MAC status.
 *    >spimac_status;   - Retrieve and display the SPI MAC state.
 *    >version;         - Retrieve and display the device version.
 *    >firmware;        - Retrieve and display the firmware version of the module.
 *    >address;         - Read and display the device address.
 *    >write_addr;      - Write a new device address to the module.
 *    >seckey;          - Read and display the security key.
 *    >write_seckey;    - Write a new security key to the module.
 *    >id;              - Read and display the device ID.
 *    >write_id;        - Write a new device ID to the module.
 *    >tcxo;            - Retrieve and display the TCXO (Temperature Compensated Crystal
 *                        Oscillator) status.
 *    >write_tcxo;      - Write new TCXO settings.
 *    >sn;              - Retrieve and display the device serial number.
 *    >read_radio_conf; - Read and display the current radio configuration.
 *    >write_radio_conf;- Write a new radio configuration to the module.
 *    >save_radio_conf; - Save the current radio configuration to non-volatile memory.
 *    >read_kmac;       - Read and display the KMAC (Key Management and Control) profile.
 *    >reload_kmac;     - Reload the KMAC configuration.
 *    >read_lpm;        - Read and display the current Low Power Mode (LPM) setting.
 *    >write_lpm;       - Write a new Low Power Mode setting.
 *    >uplink;          - Send an uplink message. By default, this forwards the same LDA2 
 *                        message every 50 seconds with a ±10% jitter.
 *
 * Hardware Setup:
 *    - Argos SMD Wings combined with an STM32WL SMD module for Arctic communications.
 *    - Feather nRF52840 serves as the main controller for scheduling, Bluetooth connectivity,
 *      LED control, and SPI communication with the Argos module.
 *
 * Additional Notes:
 *    - The sketch automatically reconfigures NFC pins (if necessary) to ensure they are used
 *      as general-purpose I/O.
 *    - The Serial port is used for debugging and for receiving commands; if USB communication
 *      is preferred, replace the `traceOutput` macro with `Serial` and enable the Serial port
 *      initialization in setup().
 *    - The delay between two commands is depends of the DEBUG mode enabled or not on the SMD module.
 *
 * License: MIT License. See the accompanying LICENSE file for details.
 **********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <variant.h>

/**
 * @brief Instance of the NeoPixel LED.
 *
 * This NeoPixel is configured for a single LED on pin 8 using the NEO_GRB color ordering
 * and an 800 KHz data signal.
 */
Adafruit_NeoPixel onePixel = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);

//============================================================================
// SPI Settings
//============================================================================
/**
 * @brief SPI pin definitions.
 */
#define SPI_SCK  (26)    ///< Default SPI Clock (SCK) pin.
#define SPI_MOSI (24)    ///< Default SPI Master Out Slave In (MOSI) pin.
#define SPI_MISO (25)    ///< Default SPI Master In Slave Out (MISO) pin.
#define SPI_CS   (2)     ///< Custom Chip Select (CS) pin.

/**
 * @brief PA_PSEL pin.
 *
 * This pin is used to select the PA. It is normally managed by the SMD.
 */
#define PA_PSEL_PIN (10)

//============================================================================
// Global Variables
//============================================================================
unsigned int txPeriod = 50000;         ///< Base transmission period (in milliseconds).
unsigned int nextTx = 50000;           ///< Next transmission period (with jitter).
unsigned int jitterPerc = 10;          ///< Jitter percentage applied to txPeriod.
String TXmessage = "000000000000000000000000000000000000000000000000"; ///< Default TX message.

unsigned long startComm = 0;           ///< Timestamp for start of communication.
unsigned long endComm = 0;             ///< Timestamp for end of communication.
unsigned long lastTime = 0;            ///< Timestamp of the last periodic event.
unsigned long startTime = 0;           ///< System start time in milliseconds.

#define traceOutput Serial            ///< Macro for debug output via Serial.

bool cmdInProgress = false;            ///< Flag indicating if a command is currently being received.
int charIdx = 0;                       ///< Index for accumulating command characters.
char InCmd[255];                       ///< Buffer for incoming command characters.

//============================================================================
// Setup and Main Loop
//============================================================================

/**
 * @brief Initializes hardware peripherals including NeoPixel, Serial, and SPI.
 *
 * This function performs initial hardware configuration:
 * - It initializes the NeoPixel LED and sets it to a green indicator.
 * - It starts the Serial port for debug output.
 * - It configures the SPI interface and checks if NFC pins need to be reconfigured.
 * - It prints an initialization message and a help menu.
 */
void setup() {
  // Initialize NeoPixel for visual startup indication.
  onePixel.begin();
  onePixel.clear();
  onePixel.setBrightness(20);
  onePixel.setPixelColor(0, 0, 255, 0);  // Set LED to green.
  onePixel.show();

  // Initialize Serial for debug output.
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  // Configure NFC pin settings if required.
  if (NRF_UICR->NFCPINS != 0xFFFFFF00) {
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;  // Enable write to UICR.
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    
    NRF_UICR->NFCPINS = 0xFFFFFF00;  // Disable NFC and use GPIO.
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;  // Lock UICR (read-only).
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);

    Serial.println("D2 configured as GPIO.");
    NVIC_SystemReset();  // Reset the board to apply changes.
  } else {
    Serial.println("D2/NFC2 is already set to GPIO.");
  }
  
  // Initialize SPI.
  SPI.begin();
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);  // Set CS high (inactive).

  // Print initialization message and display available commands.
  traceOutput.println("SPI/UART Wrapper Initialized. Send AT commands over Serial.");
  print_help();
  
  // Configure PA_PSEL pin (if applicable).
  pinMode(PA_PSEL_PIN, OUTPUT); // May be managed by the SMD module.
  digitalWrite(PA_PSEL_PIN, HIGH);

  // Record the start time.
  startTime = millis();
}

/**
 * @brief Main loop for processing SPI events and Serial commands.
 *
 * This loop checks for periodic events (such as transmissions) and processes
 * incoming Serial data by calling the handleSerial() function.
 */
void loop() {
  // Check if it's time for the next transmission.
  if (millis() - lastTime > nextTx) {
    // Uncomment the following line to enable uplink transmission.
    // smd_uplink();
    nextTx = txPeriod + random(-txPeriod * jitterPerc / 100, txPeriod * jitterPerc / 100 + 1);
    lastTime = millis();
  }
  
  // Process any incoming Serial data.
  handleSerial();
}

//============================================================================
// Serial Handling Functions
//============================================================================

/**
 * @brief Handles incoming Serial data and accumulates commands.
 *
 * This function reads characters from the Serial input. It starts accumulating a command when
 * the '>' character is received and finishes the command upon receiving a ';' character. The complete
 * command is then processed.
 */
void handleSerial() {
  while (traceOutput.available()) {
    uint8_t ch = traceOutput.read();
    if (cmdInProgress) {
      InCmd[charIdx++] = (char)ch;
    }
    if (ch == '>') {
      // Start of new command.
      cmdInProgress = true;
      charIdx = 0;
    } else if (ch == ';') {
      // End of command: process it.
      cmdInProgress = false;
      process_cmd(String(InCmd));
      memset(InCmd, ' ', sizeof(InCmd));
      InCmd[0] = '\0';
    }
  }
}

/**
 * @brief Processes a received Serial command.
 *
 * This function compares the received command string against known commands and invokes
 * the corresponding SPI functions to execute the command.
 *
 * @param cmd The command string to process.
 */
void process_cmd(String cmd) {
  if (cmd.startsWith("?;")) {
    print_help();
  } else if (cmd.startsWith("ping;")) {
    smd_ping();
  } else if (cmd.startsWith("spi_status;")) {
    smd_spi_status();
  } else if (cmd.startsWith("mac_status;")) {
    smd_mac_status();
  } else if (cmd.startsWith("spimac_status;")) {
    smd_read_spimac_state();
  } else if (cmd.startsWith("version;")) {
    smd_version();
  } else if (cmd.startsWith("firmware;")) {
    smd_firmware();
  } else if (cmd.startsWith("address;")) {
    smd_address();
  } else if (cmd.startsWith("write_addr;")) {
    smd_write_address();
  } else if (cmd.startsWith("seckey;")) {
    smd_seckey();
  } else if (cmd.startsWith("write_seckey;")) {
    smd_write_seckey();
  } else if (cmd.startsWith("id;")) {
    smd_id();
  } else if (cmd.startsWith("write_id;")) {
    smd_write_id();
  } else if (cmd.startsWith("tcxo;")) {
    smd_tcxo();
  } else if (cmd.startsWith("write_tcxo;")) {
    smd_write_tcxo();
  } else if (cmd.startsWith("sn;")) {
    smd_sn();
  } else if (cmd.startsWith("read_radio_conf;")) {
    smd_read_radio_conf();
  } else if (cmd.startsWith("write_radio_conf;")) {
    smd_write_radio_conf();
  } else if (cmd.startsWith("save_radio_conf;")) {
    smd_save_radio_conf();
  } else if (cmd.startsWith("read_kmac;")) {
    smd_read_kmac();
  } else if (cmd.startsWith("reload_kmac;")) {
    smd_reload_conf();
  } else if (cmd.startsWith("read_lpm;")) {
    smd_read_lpm();
  } else if (cmd.startsWith("write_lpm;")) {
    smd_write_lpm();
  } else if (cmd.startsWith("uplink;")) {
    smd_uplink();
  } else {
    traceOutput.println("Unknown command.");
  }
}

//============================================================================
// SPI Communication Functions
//============================================================================

/**
 * @brief Writes a buffer of data via SPI.
 *
 * This function initiates an SPI transaction, sends the data from the provided buffer,
 * and then terminates the transaction.
 *
 * @param buf Pointer to the data buffer to transmit.
 * @param size Number of bytes to send.
 */
void spi_write(uint8_t *buf, uint16_t size) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  
  SPI.transfer(buf, NULL, size);  // Transmit the data buffer.

  digitalWrite(SPI_CS, HIGH);
  SPI.endTransaction();
}

/**
 * @brief Reads a specified number of bytes via SPI.
 *
 * This function initiates an SPI transaction, reads data into the provided buffer,
 * and then terminates the transaction.
 *
 * @param buf Pointer to the buffer where received data will be stored.
 * @param size Number of bytes to read.
 */
void spi_read(uint8_t *buf, uint16_t size) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  uint8_t tx_buf[size] = {0};
  SPI.transfer(tx_buf, buf, size);  // Read data into the buffer.

  digitalWrite(SPI_CS, HIGH);
  SPI.endTransaction();
}

//============================================================================
// SPI Command Functions
//============================================================================

/**
 * @brief Sends a ping command via SPI and checks the response.
 *
 * This function sends a ping command (0x02) to the SPI device and reads the response.
 * If the response equals 1, a confirmation message is printed.
 */
void smd_ping() {
  uint8_t spiRxBuffer[1] = {0};  // Receive buffer.
  uint8_t spiTxBuffer[1] = {0x02};  // Ping command.
 
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  delay(200);
  spi_read(spiRxBuffer, sizeof(spiRxBuffer));

  traceOutput.print("Received: ");
  traceOutput.println(spiRxBuffer[0], DEC);

  if (spiRxBuffer[0] == 1) {
    traceOutput.println("Ping received");
  }
}

/**
 * @brief Retrieves the MAC status via SPI.
 *
 * This function sends a MAC status command (0x03) and prints the received status.
 */
void smd_mac_status() {
  uint8_t spiRxBuffer[1] = {0};  // Receive buffer.
  uint8_t spiTxBuffer[1] = {0x03};  // MAC status command.
 
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  delay(200);
  spi_read(spiRxBuffer, sizeof(spiRxBuffer));
  
  traceOutput.print("MAC status: ");
  traceOutput.println(spiRxBuffer[0], DEC);
}

/**
 * @brief Retrieves the SPI status via SPI.
 *
 * This function sends a SPI status command (0x04) and prints the received status.
 */
void smd_spi_status() {
  uint8_t spiRxBuffer[1] = {0};  // Receive buffer.
  uint8_t spiTxBuffer[1] = {0x04};  // SPI status command.
 
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  delay(200);
  spi_read(spiRxBuffer, sizeof(spiRxBuffer));
  
  traceOutput.print("SPI status: ");
  traceOutput.println(spiRxBuffer[0], DEC);
}

/**
 * @brief Retrieves the device version via SPI.
 *
 * This function sends a version command (0x05) and prints the received version.
 */
void smd_version() {
  uint8_t spiRxBuffer[1] = {0};  // Receive buffer.
  uint8_t spiTxBuffer[1] = {0x05};  // Version command.
 
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  delay(200);
  spi_read(spiRxBuffer, sizeof(spiRxBuffer));

  traceOutput.print("VERSION: ");
  traceOutput.println(spiRxBuffer[0], DEC);
}

/**
 * @brief Retrieves the firmware version via SPI.
 *
 * This function sends a firmware version command (0x06) and prints the resulting version string.
 */
void smd_firmware() {
  uint8_t spiRxBuffer[32] = {0};  // Buffer for firmware version.
  uint8_t spiTxBuffer[1] = {0x06};  // Firmware command.
  
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  delay(200);
  spi_read(spiRxBuffer, sizeof(spiRxBuffer));
  
  traceOutput.print("Firmware Version: ");
  for (int i = 0; i < sizeof(spiRxBuffer); i++) {
    if (spiRxBuffer[i] == '\0') break;
    traceOutput.print((char)spiRxBuffer[i]);
  }
  traceOutput.println();
}

/**
 * @brief Retrieves the device address via SPI.
 *
 * This function sends an address command (0x07) and prints the received address in hexadecimal format.
 */
void smd_address() {
  uint8_t spiRxBuffer[4] = {0};  // Buffer for address.
  uint8_t spiTxBuffer[1] = {0x07};  // Address command.
  
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  delay(200);
  spi_read(spiRxBuffer, sizeof(spiRxBuffer));
  
  traceOutput.print("Address Version: ");
  for (int i = 0; i < sizeof(spiRxBuffer); i++) {
      traceOutput.print("0x");
      if (spiRxBuffer[i] < 0x10) {
          traceOutput.print("0");
      }
      traceOutput.print(spiRxBuffer[i], HEX);
      traceOutput.print(" ");
  }
  traceOutput.println();
}

/**
 * @brief Retrieves the security key via SPI.
 *
 * This function sends a security key command (0x24) and prints the received key in hexadecimal format.
 */
void smd_seckey() {
  uint8_t spiRxBuffer[16] = {0};  // Buffer for security key.
  uint8_t spiTxBuffer[1] = {0x24};  // Security key command.
  
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  delay(200);
  spi_read(spiRxBuffer, sizeof(spiRxBuffer));
  
  traceOutput.print("SEC key: ");
  for (int i = 0; i < sizeof(spiRxBuffer); i++) {
      traceOutput.print("0x");
      if (spiRxBuffer[i] < 0x10) {
          traceOutput.print("0");
      }
      traceOutput.print(spiRxBuffer[i], HEX);
      traceOutput.print(" ");
  }
  traceOutput.println();
}

/**
 * @brief Writes a new security key via SPI.
 *
 * This function sends a write security key command by first sending 0x25 as a request,
 * then 0x26 with the new key data.
 */
void smd_write_seckey() {
  uint8_t spiTxBuffer[17] = {0};  // Buffer for security key write (1 command + 16 key bytes).
  uint8_t spiRxBuffer[1] = {0};     // Dummy receive buffer.
  
  spiTxBuffer[0] = 0x25;  // Write security key request.
  spi_write(spiTxBuffer, 1);
  delay(200);
  spiTxBuffer[0] = 0x26;  // Write security key command.
  uint8_t addr[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                      0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
  memcpy(&(spiTxBuffer[1]), addr, sizeof(spiTxBuffer)-1);
  
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  traceOutput.println("sec key write");
}

/**
 * @brief Retrieves the SPI MAC state via SPI.
 *
 * This function sends a command (0x27) to read the SPI MAC state and prints both the SPI and MAC statuses.
 */
void smd_read_spimac_state() {
  uint8_t spiTxBuffer[1] = {0};    // Command buffer.
  uint8_t spiRxBuffer[2] = {0, 0};   // Buffer for SPI and MAC statuses.
  
  spiTxBuffer[0] = 0x27;  // SPI MAC state command.
  spi_write(spiTxBuffer, 1);
  delay(200);
  spi_read(spiRxBuffer, sizeof(spiRxBuffer));
  
  traceOutput.print("SPI status: ");
  traceOutput.print(spiRxBuffer[0]);
  traceOutput.print(" MAC status: ");
  traceOutput.println(spiRxBuffer[1]);
}

/**
 * @brief Retrieves the device ID via SPI.
 *
 * This function sends a device ID command (0x08), combines the received bytes into a 32-bit integer,
 * and prints the result in both decimal and hexadecimal formats.
 */
void smd_id() {
  uint8_t spiRxBuffer[4] = {0};   // Buffer for device ID.
  uint8_t spiTxBuffer[1] = {0x08};  // Device ID command.
  
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  delay(200);
  spi_read(spiRxBuffer, sizeof(spiRxBuffer));

  uint32_t receivedInt = 0;
  for (int i = 0; i < sizeof(spiRxBuffer); i++) {
     receivedInt |= spiRxBuffer[i] << (8 * i);
  }

  traceOutput.print("ID (Decimal): ");
  traceOutput.println(receivedInt);
  traceOutput.print("ID (Hex): 0x");
  traceOutput.println(receivedInt, HEX);
}

/**
 * @brief Writes a new device ID via SPI.
 *
 * This function sends a write ID command by first sending 0x20 as a request,
 * then sending 0x21 followed by a 32-bit integer representing the new device ID.
 */
void smd_write_id() {
  uint8_t spiTxBuffer[5] = {0};  // Buffer for device ID write (1 command + 4 bytes).
  uint8_t spiRxBuffer[1] = {0};    // Dummy receive buffer.
  
  spiTxBuffer[0] = 0x20;  // Write ID request command.
  spi_write(spiTxBuffer, 1);
  delay(200);
  spiTxBuffer[0] = 0x21;  // Write ID command.
  uint32_t id = 123456;
  memcpy(&(spiTxBuffer[1]), &id, sizeof(spiTxBuffer)-1);
  
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  traceOutput.println("ID write");
}

/**
 * @brief Writes a new device address via SPI.
 *
 * This function sends a write address command by first sending 0x22 as a request,
 * then sending 0x23 followed by a 4-byte address.
 */
void smd_write_address() {
  uint8_t spiTxBuffer[5] = {0};  // Buffer for address write (1 command + 4 bytes).
  uint8_t spiRxBuffer[1] = {0};    // Dummy receive buffer.
  
  spiTxBuffer[0] = 0x22;  // Write address request command.
  spi_write(spiTxBuffer, 1);
  delay(200);
  spiTxBuffer[0] = 0x23;  // Write address command.
  uint8_t addr[4] = {0x00, 0x11, 0x22, 0x33};
  memcpy(&(spiTxBuffer[1]), addr, sizeof(spiTxBuffer)-1);
  
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  traceOutput.println("addr write");
}

/**
 * @brief Retrieves the TCXO status via SPI.
 *
 * This function sends a TCXO status command (0x28) and converts the received 4 bytes into
 * a 32-bit integer representing the TCXO time in milliseconds.
 */
void smd_tcxo() {
  uint8_t spiRxBuffer[4] = {0};  // Buffer for TCXO time.
  uint8_t spiTxBuffer[1] = {0x28};  // TCXO status command.
  
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  delay(200);
  spi_read(spiRxBuffer, sizeof(spiRxBuffer));

  uint32_t receivedInt = 0;
  for (int i = 0; i < sizeof(spiRxBuffer); i++) {
     receivedInt |= spiRxBuffer[i] << (8 * i);
  }

  traceOutput.print("TCXO time is (ms): ");
  traceOutput.println(receivedInt);
}

/**
 * @brief Writes new TCXO settings via SPI.
 *
 * This function sends a TCXO write request (0x29) followed by a command (0x2A) with a 32-bit value.
 */
void smd_write_tcxo() {
  uint8_t spiTxBuffer[5] = {0};  // Buffer for TCXO write (1 command + 4 bytes).
  uint8_t spiRxBuffer[1] = {0};    // Dummy receive buffer.
  
  spiTxBuffer[0] = 0x29;  // TCXO write request.
  spi_write(spiTxBuffer, 1);
  delay(200);
  spiTxBuffer[0] = 0x2A;  // TCXO write command.
  uint32_t tcxo = 3000;
  memcpy(&(spiTxBuffer[1]), &tcxo, sizeof(spiTxBuffer)-1);
  
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  traceOutput.println("TCXO write");
}

/**
 * @brief Retrieves the serial number via SPI.
 *
 * This function sends a serial number command (0x09) and prints the received serial number as a string.
 */
void smd_sn() {
  uint8_t spiRxBuffer[16] = {0};  // Buffer for serial number.
  uint8_t spiTxBuffer[1] = {0x09};  // Serial number command.
  
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  delay(200);
  spi_read(spiRxBuffer, sizeof(spiRxBuffer));
  
  traceOutput.print("SN Version: ");
  for (int i = 0; i < sizeof(spiRxBuffer); i++) {
    if (spiRxBuffer[i] == '\0') break;
    traceOutput.print((char)spiRxBuffer[i]);
  }
  traceOutput.println();
}

/**
 * @brief Retrieves the radio configuration via SPI.
 *
 * This function sends a radio configuration command (0x0A), decodes the received values for minimum
 * frequency, maximum frequency, RF level, and modulation, then prints them.
 */
void smd_read_radio_conf() {
  uint8_t spiRxBuffer[13] = {0};  // Buffer for radio configuration data.
  uint8_t spiTxBuffer[1] = {0x0A};  // Radio configuration command.
  
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  delay(200);
  spi_read(spiRxBuffer, sizeof(spiRxBuffer));

  uint32_t min_frequency = (spiRxBuffer[3] << 24) | (spiRxBuffer[2] << 16) |
                           (spiRxBuffer[1] << 8)  | spiRxBuffer[0];
  uint32_t max_frequency = (spiRxBuffer[7] << 24) | (spiRxBuffer[6] << 16) |
                           (spiRxBuffer[5] << 8)  | spiRxBuffer[4];
  int8_t rf_level = spiRxBuffer[8];
  uint8_t modulation = spiRxBuffer[9];

  traceOutput.print("Min Frequency: ");
  traceOutput.println(min_frequency);
  traceOutput.print("Max Frequency: ");
  traceOutput.println(max_frequency);
  traceOutput.print("RF Level: ");
  traceOutput.println(rf_level);
  traceOutput.print("Modulation: ");
  switch (modulation) {
      case 2: traceOutput.println("KNS_TX_MOD_LDA2"); break;
      case 3: traceOutput.println("KNS_TX_MOD_LDA2L"); break;
      case 4: traceOutput.println("KNS_TX_MOD_VLDA4"); break;
      case 5: traceOutput.println("KNS_TX_MOD_HDA4"); break;
      case 6: traceOutput.println("KNS_TX_MOD_LDK"); break;
      default: traceOutput.println("Unknown Modulation");
  }
}

/**
 * @brief Writes a new radio configuration via SPI.
 *
 * This function sends a radio configuration write command (0x0B) followed by a configuration string.
 */
void smd_write_radio_conf() {
  uint8_t spiRxBuffer[14] = {0};  // Dummy receive buffer.
  uint8_t spiTxBuffer[33] = {0x0B};  // Buffer for radio configuration write.
  
  const char* conf_LDA2 = "44cd3a299068292a74d2126f3402610d";
  const char* conf_LDA2L = "bd176535b394a665bd86f354c5f424fb";
  const char* conf_VLDA4 = "efd2412f8570581457f2d982e76d44d7";
  const char* conf_LDK   = "41bc11b8980df01ba8b4b8f41099620b";
  
  spi_write(spiTxBuffer, 1);
  delay(200);
  spiTxBuffer[0] = 0x0C;  // Command for writing radio configuration.
  memcpy(&(spiTxBuffer[1]), conf_LDK, sizeof(spiTxBuffer)-1);
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  traceOutput.println("rconf write");
}

/**
 * @brief Saves the current radio configuration via SPI.
 *
 * This function sends a save radio configuration command (0x0D) and checks for a confirmation response.
 */
void smd_save_radio_conf() {
  uint8_t spiRxBuffer[1] = {0};  // Buffer for confirmation.
  uint8_t spiTxBuffer[1] = {0x0D};  // Save radio configuration command.
 
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  delay(200);
  spi_read(spiRxBuffer, sizeof(spiRxBuffer));

  traceOutput.print("Received: ");
  traceOutput.println(spiRxBuffer[0], DEC);

  if (spiRxBuffer[0] == 1) {
    traceOutput.println("Conf saved");
  }
  
  digitalWrite(SPI_CS, HIGH);
  SPI.endTransaction();
}

/**
 * @brief Retrieves the KMAC profile via SPI.
 *
 * This function sends a KMAC read command (0x0E), decodes the returned profile values,
 * and prints both the raw data and decoded information.
 */
void smd_read_kmac() {
  uint8_t spiRxBuffer[8] = {0};  // Buffer for KMAC data.
  uint8_t spiTxBuffer[1] = {0x0E};  // KMAC read command.
  
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  delay(200);
  spi_read(spiRxBuffer, sizeof(spiRxBuffer));

  uint8_t profil_id = spiRxBuffer[0];
  uint8_t profil_ptr = spiRxBuffer[1];
  uint8_t profil_retx_nb = spiRxBuffer[2];
  uint8_t profil_nb_parallel = spiRxBuffer[3];

  uint32_t profil_period_s = (spiRxBuffer[7] << 24) | (spiRxBuffer[6] << 16) |
                             (spiRxBuffer[5] << 8)  | spiRxBuffer[4];

  traceOutput.println("Raw SPI Rx Buffer:");
  for (int i = 0; i < sizeof(spiRxBuffer); i++) {
      traceOutput.print("0x");
      if (spiRxBuffer[i] < 16) traceOutput.print("0");
      traceOutput.print(spiRxBuffer[i], HEX);
      if (i < sizeof(spiRxBuffer) - 1) traceOutput.print(" ");
  }
  traceOutput.println();

  traceOutput.println("Decoded KMAC Profile Values:");
  traceOutput.print("Profile ID: ");
  traceOutput.println(profil_id);
  traceOutput.print("Profile Pointer: ");
  traceOutput.println(profil_ptr);
  traceOutput.print("Profile Retransmission Count: ");
  traceOutput.println(profil_retx_nb);
  traceOutput.print("Number of Parallel Profiles: ");
  traceOutput.println(profil_nb_parallel);
  traceOutput.print("Profile Period (s): ");
  traceOutput.println(profil_period_s);
}

/**
 * @brief Reloads the KMAC configuration via SPI.
 *
 * This function reloads the KMAC configuration by first setting it to 0, waiting,
 * then setting it to 1, to trigger a configuration reload.
 */
void smd_reload_conf() {
  uint8_t spiRxBuffer[1] = {0};  // Buffer for response.
  uint8_t spiTxBuffer[2] = {0x0F};  // Buffer for KMAC reload command.

  traceOutput.println("SET KMAC to 0");
  spi_write(spiTxBuffer, 1);
  delay(200);
  spiTxBuffer[0] = 0x10;
  spiTxBuffer[1] = 0;
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  
  delay(2000);
  traceOutput.println("SET KMAC to 1");
  spiTxBuffer[0] = 0x0F;
  spi_write(spiTxBuffer, 1);
  delay(200);
  spiTxBuffer[0] = 0x10;
  spiTxBuffer[1] = 1;
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  traceOutput.println("Radio_conf write");
}

/**
 * @brief Retrieves the low power mode (LPM) setting via SPI.
 *
 * This function sends a command (0x11) to read the LPM setting and prints the result.
 */
void smd_read_lpm() {
  uint8_t spiRxBuffer[1] = {0};  // Buffer for LPM.
  uint8_t spiTxBuffer[1] = {0x11};  // LPM read command.
 
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  delay(200);
  spi_read(spiRxBuffer, sizeof(spiRxBuffer));
  
  traceOutput.print("LPM: ");
  traceOutput.println(spiRxBuffer[0], DEC);
}

/**
 * @brief Writes a new low power mode (LPM) setting via SPI.
 *
 * This function sends a command to write a new LPM setting. In this example, the LPM is set to 0.
 */
void smd_write_lpm() {
  uint8_t spiRxBuffer[1] = {0};  // Dummy receive buffer.
  uint8_t spiTxBuffer[2] = {0x12};  // Buffer for LPM write.
  
  traceOutput.println("Write LPM NONE");
  spi_write(spiTxBuffer, 1);
  delay(200);
  spiTxBuffer[0] = 0x13;
  spiTxBuffer[1] = 0;
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
}

/**
 * @brief Sends a string command via SPI.
 *
 * This function converts the provided String into a character array and transmits it over SPI.
 *
 * @param cmd The command string to be sent.
 */
void sendSPI(String cmd) {
  uint8_t txBuffer[64] = {0};
  cmd.toCharArray((char *)txBuffer, cmd.length() + 1);

  digitalWrite(SPI_CS, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(txBuffer, (size_t) cmd.length());
  SPI.endTransaction();
  digitalWrite(SPI_CS, HIGH);

  traceOutput.print("Sent via SPI: ");
  traceOutput.println(cmd);
}

/**
 * @brief Test function to send a simple value via SPI.
 *
 * This function transmits the value 1 via SPI using a higher clock speed, then prints the sent value.
 */
void testSPI() {
  uint8_t test = 1;
  
  digitalWrite(SPI_CS, LOW);
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(1);
  SPI.endTransaction();
  digitalWrite(SPI_CS, HIGH);

  traceOutput.print("Sent via SPI: 1");
}

/**
 * @brief Sends an uplink message via SPI.
 *
 * This function initiates an uplink transmission by sending a series of SPI commands:
 * - It starts the uplink with command 0x14.
 * - It specifies the payload size with command 0x15.
 * - It transmits the actual payload with command 0x16.
 */
void smd_uplink() {
  uint8_t spiRxBuffer[1] = {0};  // Dummy receive buffer.
  uint8_t spiTxBuffer[20] = {0xFF};  // Buffer for uplink message.
  
  traceOutput.println("Forward TX message");
  
  spiTxBuffer[0] = 0x14;  // Uplink initiation command.
  spi_write(spiTxBuffer, 1);
  delay(300);
  
  spiTxBuffer[0] = 0x15;  // TX data size command.
  spiTxBuffer[1] = 0;
  spiTxBuffer[2] = sizeof(spiTxBuffer) - 1;
  spi_write(spiTxBuffer, 3);
  delay(300);
  
  memset(spiTxBuffer, 0xFF, sizeof(spiTxBuffer));
  spiTxBuffer[0] = 0x16;  // TX payload command.
  spi_write(spiTxBuffer, sizeof(spiTxBuffer));
  delay(500);
}

/**
 * @brief Prints the list of available SPI commands.
 *
 * This function outputs a help message to the debug console that lists all available commands.
 * Each command is described along with its purpose.
 *
 * @note Each command must be terminated with a semicolon (';').
 */
void print_help() {
  traceOutput.println("Available Commands:");
  traceOutput.println(">?; => Display this help message");
  traceOutput.println(">ping; => Send a ping command to the device");
  traceOutput.println(">spi_status; => Retrieve the current SPI status");
  traceOutput.println(">mac_status; => Retrieve the current MAC status");
  traceOutput.println(">spimac_status; => Retrieve the SPI MAC state");
  traceOutput.println(">version; => Retrieve the device version");
  traceOutput.println(">firmware; => Retrieve the firmware version");
  traceOutput.println(">address; => Read the device address");
  traceOutput.println(">write_addr; => Write a new device address");
  traceOutput.println(">seckey; => Read the security key");
  traceOutput.println(">write_seckey; => Write a new security key");
  traceOutput.println(">id; => Read the device ID");
  traceOutput.println(">write_id; => Write a new device ID");
  traceOutput.println(">tcxo; => Retrieve the TCXO status");
  traceOutput.println(">write_tcxo; => Write new TCXO settings");
  traceOutput.println(">sn; => Retrieve the serial number");
  traceOutput.println(">read_radio_conf; => Read the radio configuration");
  traceOutput.println(">write_radio_conf; => Write the radio configuration");
  traceOutput.println(">save_radio_conf; => Save the current radio configuration");
  traceOutput.println(">read_kmac; => Read the KMAC profile");
  traceOutput.println(">reload_kmac; => Reload the KMAC configuration");
  traceOutput.println(">read_lpm; => Read the low power mode setting");
  traceOutput.println(">write_lpm; => Write the low power mode setting");
  traceOutput.println(">uplink; => Send an uplink message");
}
