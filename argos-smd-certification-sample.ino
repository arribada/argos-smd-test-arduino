/**********************************************************************
 This example is used to test the HorizonLinks FeatherWings with argos
 SMD module. This application is based on the 
 Bluefruit app BLE_UART example

 Hardware used:
    - argos-smd-wings + STM32WL SMD module: Arctic communication (Serial)
    - Feather nRF52840: Main scheduler, Bluetooth, LED control, 
      Serial communication with Wings_Breakout and Wings_GNSS
    
 By default, this program forwards the same LDA2 message every 50 seconds with a jitter 
 of ±10% (uniformly distributed between 45 and 55 seconds).

 Download the Bluefruit Connect app and connect to your device.

    Blue LED blinking: Waiting for Bluetooth connection
    Fixed blue LED: Connected

 Go to the UART tab, where you can see the device trace and enter commands. 
 Type >?; for the following help:

    Please enter commands via Bluetooth with the following format:
        Start with the character '>'
        End with the character ';'
    Commands available:
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
            >conf_reload; => Reload MAC SMD configuration
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
            

 Note: The USB virtual Serial (Serial) is not used in this example. 
 If you want to replace the BLE app and use only USB communication, 
 replace the `traceOutput` variable from `bleuart` to `Serial`. You also need 
 to uncomment the Serial initialization in the `setup()` function.
 
 Note2: The following pins have been adapted in the `variant.h` file 
 located at:
 {ArduinoPath}\Arduino15\packages\adafruit\hardware\nrf52\1.5.0\variants
 \feather_nrf52840_express\variant.h
 If you don't want to change it, redefine the correct pin for SerialSMD

 #define PIN_SERIAL1_RX       (12)
 #define PIN_SERIAL1_TX       (13)

 #define PIN_SERIAL2_RX       (1)
 #define PIN_SERIAL2_TX       (0)

 MIT license, check LICENSE for more information.
 All text above, and the splash screen below, must be included in
 any redistribution.
*********************************************************************/

/* Include */
#include <Arduino.h>
#include <Wire.h>
#include <Uart.h>
#include <variant.h>

#include <bluefruit.h>
#include <Adafruit_NeoPixel.h>  //  Library that provides NeoPixel functions
#include <SoftwareSerial.h>

#include "Adafruit_TinyUSB.h"

/* Manage Battery defines and variables */
#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096

#ifdef NRF52840_XXAA
#define VBAT_DIVIDER      (0.5F)          // 150K + 150K voltage divider on VBAT
#define VBAT_DIVIDER_COMP (2.0F)          // Compensation factor for the VBAT divider
#else
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider
#endif

#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)
/*********************************************************************/

// -- Create a NeoPixel object called onePixel that addresses 1 pixel in pin 8
Adafruit_NeoPixel onePixel = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);
/*********************************************************************/

// Manage SMD UART link
#define STM_UART_RX (PIN_SERIAL2_RX)
#define STM_UART_TX (PIN_SERIAL2_TX)
#define SerialSTM Serial2
#define PA_PSEL_PIN (10) // Depending if it's managed by the SMD (by default yes)

// Manage Argos periodic message
unsigned int txPeriod = 50000;
unsigned int nextTx = 50000;
unsigned int jitterPerc = 10;
unsigned long lastTimeTx = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
unsigned long startTimeTx = 0; //Used to calc the actual update rate.

unsigned long startComm = 0; //Simple local timer. Calculate time to send argos message
unsigned long endComm = 0;   //Used to calc the actual update rate.



String TXpayload =       "000000000000000000000000000000000000000000000000";
String TXpayload_LDA2 =  "000000000000000000000000000000000000000000000000";
String TXpayload_LDA2L = "000000000000000000000000000000000000000000000000";
String TXpayload_LDK =   "00000000000000000000000000000000000000";
String TXpayload_VLDA4 = "000000";

String CWParams = "";
String LPM_Mode = "";
/*********************************************************************/



// BLE Service
BLEDfu bledfu;    // OTA DFU service
BLEDis bledis;    // device information
BLEUart bleuart;  // uart over ble
BLEBas blebas;    //
/*********************************************************************/

// BLE Service
// change Bleuart to Serial for USB communication, uncomment Serial 
// initialization in setup()
#define traceOutput (bleuart) 


bool userCmdInProgress = false;
int userCmdIdx = 0;

char userInCmd[255];
bool testMode = true;

/**
 * @brief Initializes the hardware and sets up the required peripherals.
 *        Also initializes Bluefruit Bluetooth, sets up services, and starts advertising.
 */
void setup() {
  // Neopixel init
  onePixel.begin();            // Start the NeoPixel object
  onePixel.clear();           // Set NeoPixel color to black (0,0,0)
  onePixel.setBrightness(20);  // Affects all subsequent settings

  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
  // CDC Serial INIT for debug (through USB)
  // Disable it and use BLE instead. Free for STM32
  // Serial.begin(9600);
  // while (!Serial) {
  //   delay(10);

  //   onePixel.setPixelColor(0, 0xff, 0x00, 0); // Red Green Blue
  //   onePixel.show();
  // }
  #if CFG_DEBUG
    // Blocking wait for connection when debug mode is enabled via IDE
    while (!Serial) yield();
  #endif


  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);  // Check bluefruit.h for supported values
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Arribada Initiative");
  Bluefruit.setName("argos-smd-wings");
  bledis.setModel("Argos SMD Wings");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();

  // Serial init for STM32 SMD module Argos
  SerialSTM.begin(9600);
  while (!SerialSTM) {
    delay(10);
    traceOutput.print("Trying to configure UART STM32");
    onePixel.setPixelColor(0, 0xff, 0xff, 0);  // Red Green Blue
    onePixel.show();
  }

  traceOutput.print("Please use Adafruit's Bluefruit LE app to connect in BLE UART mode");
  print_help();
  
  //Check for next TX event:
  randomSeed(analogRead(0));
  unsigned int jitterRange = txPeriod * jitterPerc / 100;
  
  // Calculate the next transmission time with jitter
  nextTx = txPeriod + random(-jitterRange, jitterRange + 1);
  

  traceOutput.print("Next TX event (ms): ");
  traceOutput.print(nextTx);

  // TURN PSEL Pin High
  pinMode(PA_PSEL_PIN, OUTPUT); // Not necessary, managed by the SMD module
  digitalWrite(PA_PSEL_PIN, HIGH);
  startTimeTx = millis();
  
  // Since V8.2.1 KMAC profile need to be reload at startup
  smd_conf_reload();
}

/**
 * @brief Starts advertising with BLE services.
 */
void startAdv(void) {
  // Advertising packeit
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(false);
  Bluefruit.Advertising.setInterval(32, 244);  // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);    // number of seconds in fast mode
  Bluefruit.Advertising.start(60);              // 0 = Don't stop advertising after n seconds
}

/**
 * @brief Main loop function.
 *        Reads data from Serial STM32 module, processes BLEUART or Serial data,
 *        and calls functions for GPS PVT and Reed switch.
 */
void loop() {
  // Read Serial STM32 module data and forward it to traceOutput
  while (SerialSTM.available() > 0) {
    // Delay to wait for enough input, since we have a limited transmission buffer
    delay(2);
    onePixel.setPixelColor(0, 0x00, 0x00, 0xFF);
    onePixel.show();

    uint8_t buf[64];

    int count = SerialSTM.readBytes(buf, sizeof(buf));
    
    if (count > 7)
    {
      int compString = strncmp((char*)buf, " TX_DONE", 8);
      if (compString == 0)
      {
        endComm = millis();
        traceOutput.print("Time to send Satellite uplink message (ms)");
        traceOutput.print(endComm-startComm);
      }
    }
    traceOutput.write(buf, count);
    
    
    onePixel.setPixelColor(0, 0, 0xff, 0);
    onePixel.show();
  }

  // Manage periodic message if testMode is enabled
  if (millis() - lastTimeTx > nextTx)
  {
    if (testMode) {
      // Send message
      smd_uplink();
      
      // Compute next TX event
      unsigned int jitterRange = txPeriod * jitterPerc / 100;
    
      // Calculate the next transmission time with jitter
      nextTx = txPeriod + random(-jitterRange, jitterRange + 1);
    
      traceOutput.print("Next TX event (s): ");
      traceOutput.print(nextTx);
      
      lastTimeTx = millis(); //Update the timer
    }
  }

  // Read from BLEUART or Serial data and interpret CMD
  while (traceOutput.available()) {
    onePixel.setPixelColor(0, 0x00, 0x00, 0xFF);
    onePixel.show();

    uint8_t ch;
    ch = (uint8_t)traceOutput.read();
    
    if (userCmdInProgress == true) {
      userInCmd[userCmdIdx++] = (char)ch;
    }
    if (ch == '>') {
      userCmdInProgress = true;
      userCmdIdx = 0;
      //userInCmd[userCmdIdx++] = (char) ch;
    } else if (ch == ';') {
      userCmdInProgress = false;
      userCmdIdx = 0;
      process_cmd(String(userInCmd));
      memset(userInCmd, ' ', sizeof(userInCmd));
      userInCmd[0] = '\0'; // Mark it as an empty string.
    }
    onePixel.setPixelColor(0, 0, 0xff, 0);
    onePixel.show();
  }

}

/**
 * @brief Processes a received command.
 * @param cmd The command string to process.
 */
void process_cmd(String cmd) {
  onePixel.setPixelColor(0, 0, 0, 0xff);
  onePixel.show();

  if (cmd.startsWith("?;")) {
    print_help();
  } else if(cmd.startsWith("test_mode;")) {
    if (testMode == true)
    {
      traceOutput.print("Command : TestMode disabled (no periodic SAT message)");
      testMode = false;
    } else {
      traceOutput.print("Command : TestMode enabled (periodic SAT message)");
      testMode = true;
    }
  } else if (cmd.startsWith("pulse;")) {
    traceOutput.print("Command : pulse cmd received");
    smd_uplink(); 
  } else if (cmd.startsWith("conf;")) {
    traceOutput.print("Command : SMD read configuration");
    smd_conf();    
  } else if (cmd.startsWith("conf_LDA2;")) {
    traceOutput.print("Command : SMD Set radio to LDA2");
    smd_conf_set_LDA2(); 
  } else if (cmd.startsWith("conf_LDA2L;")) {
    traceOutput.print("Command : SMD Set radio to LDA2L");
    smd_conf_set_LDA2L(); 
  } else if (cmd.startsWith("conf_VLDA4;")) {
    traceOutput.print("Command : SMD Set radio to VLDA4");
    smd_conf_set_VLDA4(); 
  } else if (cmd.startsWith("conf_LDK;")) {
    traceOutput.print("Command : smd Set radio to LDK");
    smd_conf_set_LDK(); 
  } else if (cmd.startsWith("conf_SAVE;")) {
    traceOutput.print("Command : smd Save radio conf");
    smd_conf_save(); 
  }else if (cmd.startsWith("conf_reload;")) {
    traceOutput.print("Command : Reload Radio conf");
    smd_conf_reload();
  } else if (cmd.startsWith("ID;")) {
    traceOutput.print("Command : smd read ID");
    smd_read_id();     
  } else if (cmd.startsWith("SN;")) {
    traceOutput.print("Command : smd read Serial Number");
    smd_read_sn();
  } else if (cmd.startsWith("ADDR;")) {
    traceOutput.print("Command : smd read Address");
    smd_read_address();
  } else if (cmd.startsWith("FW;")) {
    traceOutput.print("Command : smd read firmware version");
    smd_read_firmware_version(); 
  } else if (cmd.startsWith("AT_VERSION;")) {
    traceOutput.print("Command : Read AT version from SMD");
    smd_read_at_version(); 
  } else if (cmd.startsWith("ping;")) {
    traceOutput.print("Command : Ping SMD module ");
    smd_ping();    
  } else if (cmd.startsWith("read_LPM;")) {
    traceOutput.print("Command : smd read Low power mode");
    smd_read_lpm();    
  } else if (cmd.startsWith("udate;")) {
    traceOutput.print("Command : smd read UTC date");
    smd_udate();  
  }else if (cmd.startsWith("CW=")) {
    CWParams = cmd.substring(cmd.indexOf('=') + 1, cmd.indexOf(';')); // Extract the message content
    traceOutput.print("Command : CW forward to SMD ");
    traceOutput.println(CWParams);
    smd_cw();
  }else if (cmd.startsWith("LPM=")) {
    LPM_Mode = cmd.substring(cmd.indexOf('=') + 1, cmd.indexOf(';')); // Extract the message content
    traceOutput.print("Command : Set LPM mode ");
    //traceOutput.println(LPM_Mode);
    smd_set_lpm();
  }else if (cmd.startsWith("msg=")) {
    
    TXpayload = cmd.substring(cmd.indexOf('=')+1, cmd.indexOf(';')); // Extract the message content
    traceOutput.print("Command : Message to forward: ");
    traceOutput.println(TXpayload);
    // Do something with the message here
  }else if (cmd.startsWith("set_period=")) {
    // Extract the new period value from the command
    String periodStr = cmd.substring(11, cmd.indexOf(';'));
    txPeriod = periodStr.toInt();
    traceOutput.print("Command : Transmission period set to ");
    traceOutput.print(txPeriod);
    traceOutput.println(" ms");
  } else if (cmd.startsWith("set_jitter=")) {
    // Extract the new jitter percentage value from the command
    String jitterStr = cmd.substring(cmd.indexOf('=') + 1 , cmd.indexOf(';'));
    jitterPerc = jitterStr.toInt();
    traceOutput.print("Command : Jitter percentage set to ");
    traceOutput.print(jitterPerc);
    traceOutput.println("%");
  }
  onePixel.setPixelColor(0, 0, 0xff, 0);
  onePixel.show();
}

/**
 * @brief Reads the firmware version of the SMD module.
 * 
 * This function sends an "AT+FW=?" command to the SMD module via the SerialSTM interface.
 * It also updates the LED color to indicate the operation and logs the command being sent.
 */
void smd_read_firmware_version() {
  onePixel.setPixelColor(0, 255, 0, 255);  // Red = 255, Green = 0, Blue = 255 purple
  onePixel.show();
  //char test_cmd[] = "AT+TX=00000000";
  String test_cmd = "AT+FW=?";
  traceOutput.print("Read Firmware version (libkineis)");
  traceOutput.print(test_cmd);
  startComm = millis();

  SerialSTM.println(test_cmd);
  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
}


/**
 * @brief Sends a ping command to the SMD module to check its responsiveness.
 * 
 * This function sends an "AT+PING=?" command to the SMD module via the SerialSTM interface.
 * It also updates the LED color to indicate the operation and logs the command being sent.
 */
void smd_ping() {
  onePixel.setPixelColor(0, 255, 0, 255);  // Red = 255, Green = 0, Blue = 255 purple
  onePixel.show();

  String test_cmd = "AT+PING=?";
  traceOutput.print("Ping SMD module");
  traceOutput.print(test_cmd);
  startComm = millis();

  SerialSTM.println(test_cmd);
  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
}
/**
 * @brief Reads the MAC address of the SMD module.
 * 
 * This function sends an "AT+ADDR=?" command to the SMD module via the SerialSTM interface.
 * It also updates the LED color to indicate the operation and logs the command being sent.
 */
void smd_read_address() {
  onePixel.setPixelColor(0, 255, 0, 255);  // Red = 255, Green = 0, Blue = 255 purple
  onePixel.show();
  //char test_cmd[] = "AT+TX=00000000";
  String test_cmd = "AT+ADDR=?";
  traceOutput.print("Read smd ADDR (MAC address)");
  traceOutput.print(test_cmd);
  startComm = millis();
 
  SerialSTM.println(test_cmd);
  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
}

/**
 * @brief Reads the serial number of the SMD module.
 * 
 * This function sends an "AT+SN=?" command to the SMD module via the SerialSTM interface.
 * It also updates the LED color to indicate the operation and logs the command being sent.
 */
void smd_read_sn() {
  onePixel.setPixelColor(0, 255, 0, 255);  // Red = 255, Green = 0, Blue = 255 purple
  onePixel.show();

  String test_cmd = "AT+SN=?";
  traceOutput.print("Read smd SN");
  traceOutput.print(test_cmd);
  startComm = millis();
 
  SerialSTM.println(test_cmd);
  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
}
/**
 * @brief Reads the unique ID of the SMD module.
 * 
 * This function sends an "AT+ID=?" command to the SMD module via the SerialSTM interface.
 * It also updates the LED color to indicate the operation and logs the command being sent.
 */
void smd_read_id() {
  onePixel.setPixelColor(0, 255, 0, 255);  // Red = 255, Green = 0, Blue = 255 purple
  onePixel.show();

  String test_cmd = "AT+ID=?";
  traceOutput.print("Read smd ID");
  traceOutput.print(test_cmd);
  startComm = millis();
 
  SerialSTM.println(test_cmd);
  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
}
/**
 * @brief Requests the current radio configuration of the SMD module.
 * 
 * This function sends an "AT+RCONF=?" command to the SMD module via the SerialSTM interface.
 * It also updates the LED color to indicate the operation and logs the command being sent.
 */
void smd_conf() {
  onePixel.setPixelColor(0, 255, 0, 255);  // Red = 255, Green = 0, Blue = 255 purple
  onePixel.show();

  String test_cmd = "AT+RCONF=?";
  traceOutput.print("Request read radio configuration");
  traceOutput.print(test_cmd);
  startComm = millis();
 
  SerialSTM.println(test_cmd);
  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
}

/**
 * @brief Reloads the radio configuration of the SMD module.
 * 
 * This function sends two "AT+KMAC" commands to the SMD module to perform a configuration reload.
 * It also updates the LED color to indicate the operation and logs the commands being sent.
 */
void smd_conf_reload() {
  onePixel.setPixelColor(0, 255, 0, 255);  // Red = 255, Green = 0, Blue = 255 purple
  onePixel.show();

  String test_cmd = "AT+KMAC=0";
  traceOutput.print(test_cmd);

  delay(2000);

  test_cmd = "AT+KMAC=1";
  traceOutput.print(test_cmd);

  startComm = millis();
 
  SerialSTM.println(test_cmd);
  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
}

/**
 * @brief Changes the radio configuration to LDA2 mode (27dBm).
 * 
 * This function sends an "AT+RCONF" command with a specific payload to the SMD module.
 * It also updates the LED color to indicate the operation and logs the command being sent.
 */
void smd_conf_set_LDA2() {
  onePixel.setPixelColor(0, 255, 0, 255);  // Red = 255, Green = 0, Blue = 255 purple
  onePixel.show();
  String test_cmd = "AT+RCONF=44cd3a299068292a74d2126f3402610d";
  // Set new payload :
  TXpayload = TXpayload_LDA2;
  traceOutput.print("Change RADIO CONF to LDA2 (27dBm)");
  traceOutput.print(test_cmd);
  startComm = millis();
 
  SerialSTM.println(test_cmd);
  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
}

/**
 * @brief Changes the radio configuration to LDA2L mode (27dBm).
 * 
 * This function sends an "AT+RCONF" command with a specific payload to the SMD module.
 * It also updates the LED color to indicate the operation and logs the command being sent.
 */
void smd_conf_set_LDA2L() {
  onePixel.setPixelColor(0, 255, 0, 255);  // Red = 255, Green = 0, Blue = 255 purple
  onePixel.show();
  TXpayload = TXpayload_LDA2;
  String test_cmd = "AT+RCONF=bd176535b394a665bd86f354c5f424fb";
  traceOutput.print("Change RADIO CONF to LDA2L (27dBm)");
  traceOutput.print(test_cmd);
  startComm = millis();
 
  SerialSTM.println(test_cmd);
  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
}

/**
 * @brief Changes the radio configuration to VLDA4 mode (22dBm).
 * 
 * This function sends an "AT+RCONF" command with a specific payload to the SMD module.
 * It also updates the LED color to indicate the operation and logs the command being sent.
 */
void smd_conf_set_VLDA4() {
  onePixel.setPixelColor(0, 255, 0, 255);  // Red = 255, Green = 0, Blue = 255 purple
  onePixel.show();
  TXpayload = TXpayload_VLDA4;
  String test_cmd = "AT+RCONF=efd2412f8570581457f2d982e76d44d7";
  traceOutput.print("Change RADIO CONF to VLDA4 (22dBm)");
  traceOutput.print(test_cmd);
  startComm = millis();
 
  SerialSTM.println(test_cmd);
  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
}
 
/**
 * @brief Changes the radio configuration to LDK mode.
 * 
 * This function sends an "AT+RCONF" command with a specific payload to the SMD module.
 * It also updates the LED color to indicate the operation and logs the command being sent.
 */
void smd_conf_set_LDK() {
  onePixel.setPixelColor(0, 255, 0, 255);  // Red = 255, Green = 0, Blue = 255 purple
  onePixel.show();

  TXpayload = TXpayload_LDK;
  String test_cmd = "AT+RCONF=41bc11b8980df01ba8b4b8f41099620b";
  traceOutput.print("Change RADIO CONF to LDK");
  traceOutput.print(test_cmd);
  startComm = millis();
  

  SerialSTM.println(test_cmd);
  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
}
  
/**
 * @brief Saves the current radio configuration of the SMD module.
 * 
 * This function sends an "AT+SAVE_RCONF=" command to the SMD module via the SerialSTM interface.
 * It also updates the LED color to indicate the operation and logs the command being sent.
 */
void smd_conf_save() {
  onePixel.setPixelColor(0, 255, 0, 255);  // Red = 255, Green = 0, Blue = 255 purple
  onePixel.show();

  String test_cmd = "AT+SAVE_RCONF=";
  traceOutput.print("Save Radio config");
  traceOutput.print(test_cmd);
  startComm = millis();
 
  SerialSTM.println(test_cmd);
  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
}
  
/**
 * @brief Sends an uplink message with the current payload.
 * 
 * This function sends an "AT+TX=" command with the TX payload to the SMD module.
 * It also updates the LED color to indicate the operation and logs the command being sent.
 */
void smd_uplink() {
  onePixel.setPixelColor(0, 255, 0, 255);  // Red = 255, Green = 0, Blue = 255 purple
  onePixel.show();

  String test_cmd = "AT+TX=" + TXpayload;
  traceOutput.print("Send uplink message");
  traceOutput.print(test_cmd);
  startComm = millis();
 
  SerialSTM.println(test_cmd);
  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
}

/**
 * @brief Sends a continuous wave (CW) request to the SMD module.
 * 
 * This function sends an "AT+CW=" command with CW parameters to the SMD module.
 * It also updates the LED color to indicate the operation and logs the command being sent.
 */
void smd_cw() {
  onePixel.setPixelColor(0, 255, 0, 255);  // Red = 255, Green = 0, Blue = 255 purple
  onePixel.show();

  String test_cmd = "AT+CW=" + CWParams;
  traceOutput.print("Send CW request");
  traceOutput.print(test_cmd);
  startComm = millis();
 
  SerialSTM.println(test_cmd);
  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
}


/**
 * @brief Reads the low-power mode (LPM) status of the SMD module.
 * 
 * This function sends an "AT+LPM=?" command to the SMD module via the SerialSTM interface.
 * It also updates the LED color to indicate the operation and logs the command being sent.
 */
void smd_read_lpm() {
  onePixel.setPixelColor(0, 255, 0, 255);  // Red = 255, Green = 0, Blue = 255 purple
  onePixel.show();

  String test_cmd = "AT+LPM=?";
  traceOutput.print("Send request to read SMD LPM mode");
  traceOutput.print(test_cmd);
  startComm = millis();
 
  SerialSTM.println(test_cmd);
  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
}

/**
 * @brief Sets the low-power mode (LPM) of the SMD module.
 * 
 * This function sends an "AT+LPM=" command with the desired LPM mode to the SMD module.
 * It also updates the LED color to indicate the operation and logs the command being sent.
 */
void smd_set_lpm() {
  onePixel.setPixelColor(0, 255, 0, 255);  // Red = 255, Green = 0, Blue = 255 purple
  onePixel.show();

  String test_cmd = "AT+LPM=" + LPM_Mode;
  traceOutput.print("Send request to set LPM mode");
  traceOutput.print(test_cmd);
  startComm = millis();
 
  SerialSTM.println(test_cmd);
  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
}

/**
 * @brief Reads the AT command version from the SMD module.
 * 
 * This function sends an "AT+VERSION=?" command to the SMD module via the SerialSTM interface.
 * It also updates the LED color to indicate the operation and logs the command being sent.
 */
void smd_read_at_version() {
  onePixel.setPixelColor(0, 255, 0, 255);  // Red = 255, Green = 0, Blue = 255 purple
  onePixel.show();

  String test_cmd = "AT+VERSION=?";
  traceOutput.print("Send a request to read AT VERSION from SMD");
  traceOutput.print(test_cmd);
  startComm = millis();
 
  SerialSTM.println(test_cmd);
  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
}

/**
 * @brief Requests the UTC date from the SMD module.
 * 
 * This function sends an "AT+UDATE=?" command to the SMD module via the SerialSTM interface.
 * It also updates the LED color to indicate the operation and logs the command being sent.
 */
void smd_udate() {
  onePixel.setPixelColor(0, 255, 0, 255);  // Red = 255, Green = 0, Blue = 255 purple
  onePixel.show();

  String test_cmd = "AT+UDATE=?";
  traceOutput.print("Send udate request");
  traceOutput.print(test_cmd);
  startComm = millis();
 
  SerialSTM.println(test_cmd);
  onePixel.setPixelColor(0, 0, 0xff, 0);  // Red Green Blue
  onePixel.show();
}

/**
 * @brief Reads the battery voltage from the analog pin.
 * 
 * This function configures the ADC to measure the battery voltage, applies necessary
 * calculations to compensate for the resistor divider, and returns the result in millivolts.
 * 
 * @return The battery voltage in millivolts.
 */
float readVBAT(void) {
  float raw;

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(PIN_VBAT);

  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
  return raw * REAL_VBAT_MV_PER_LSB;
}

/**
 * @brief Converts a battery voltage in millivolts to a percentage.
 * 
 * This function maps the input millivolts to a battery percentage
 * using predefined thresholds for different voltage ranges.
 * 
 * @param mvolts The battery voltage in millivolts.
 * @return The battery percentage (0-100%).
 */
uint8_t mvToPercent(float mvolts) {
  if(mvolts<3300)
    return 0;

  if(mvolts <3600) {
    mvolts -= 3300;
    return mvolts/30;
  }

  mvolts -= 3600;
  return 10 + (mvolts * 0.15F );  // thats mvolts /6.66666666
}

/**
 * @brief Displays a help message with available commands.
 * 
 * This function sends a detailed help message over Bluetooth,
 * explaining the syntax and the list of available commands for the user.
 */
void print_help() {
  traceOutput.println("Please enter commands via Bluetooth with the following format:");
  traceOutput.println("\t - Start with character '>' ");
  traceOutput.println("\t - End with character ';' ");
  traceOutput.println("Commands available:");
  traceOutput.println("\t - >?; => Display this help message");
  traceOutput.println("\t - >test_mode; => Enable/disable default behavior (periodic SAT message, default ON)");
  traceOutput.println("\t - >pulse; => Send Argos test message");
  traceOutput.println("\t - >conf; => Read SMD configuration");
  traceOutput.println("\t - >conf_LDA2; => Set SMD radio to LDA2 mode");
  traceOutput.println("\t - >conf_LDA2L; => Set SMD radio to LDA2L mode");
  traceOutput.println("\t - >conf_VLDA4; => Set SMD radio to VLDA4 mode");
  traceOutput.println("\t - >conf_LDK; => Set SMD radio to LDK mode");
  traceOutput.println("\t - >conf_SAVE; => Save radio conf");
  traceOutput.println("\t - >ID; => Read SMD ID");
  traceOutput.println("\t - >SN; => Read SMD Serial Number");
  traceOutput.println("\t - >ADDR; => Read SMD Address");
  traceOutput.println("\t - >FW; => Read SMD firmware version");
  traceOutput.println("\t - >AT_VERSION; => Read AT version from SMD");
  traceOutput.println("\t - >ping; => Ping SMD module");
  traceOutput.println("\t - >read_LPM; => Read SMD Low Power Mode");
  traceOutput.println("\t - >udate; => Read UTC date from SMD");
  traceOutput.println("\t - >CW=Mode,Freq,Power; => Set continuous wave mode (Mode: 0-5, Freq: in Hz, Power: in dBm)");
  traceOutput.println("\t - >LPM=Mode; => Set Low Power Mode (Mode: valid power mode value)");
  traceOutput.println("\t - >msg=Message; => Change message sent via Argos");
  traceOutput.println("\t - >set_period=Value; => Set transmission period in milliseconds");
  traceOutput.println("\t - >set_jitter=Value; => Set jitter percentage for transmission timing");
}


/**
 * @brief Callback invoked when a central device connects.
 * @param conn_handle Connection handle of the central device.
 */
void connect_callback(uint16_t conn_handle) {
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  traceOutput.print("Connected to ");
  traceOutput.println(central_name);
}


/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;
  
  traceOutput.println();
  traceOutput.print("Disconnected, reason = 0x");
  traceOutput.println(reason, HEX);
}