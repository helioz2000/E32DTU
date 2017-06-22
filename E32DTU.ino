/*************************************************************************
 * E32DTU test
 * 
 * Circuit:
 * Arduino Nano -> MAX485 board
 * 5V  -> VCC
 * GND -> GND
 * D2  -> DI TX
 * D3  -> DE TX enable
 * D4  -> !RE RX enable (NOT)
 * D5  -> RO RX
 * 
 * MAX485 board -> E32-DTU-1W
 * A -> 485A
 * B -> 485B
 * Power supply via plug pack (8-24V)
 * 
 * E32-DTU:
 * dip switches:
 * 1   2
 * OFF  OFF - Config mode + sleep mode
 * ON   OFF - Receive mode only (power saving)
 * OFF  ON  - Wake Up on Receive (WOR) mode, TX sends pre-amble to wake up other end
 * ON   ON  - Transparent transmission (normal operatation)
 * 
 *************************************************************************/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "debug.h"

#define LED LED_BUILTIN

// ** Debug
#define DEBUG         // comment out to disable debugging
#ifdef DEBUG
#define DEBUG_LEVEL_DEFAULT L_INFO;
debugLevels currentDebugLevel;
#endif

#define MAX485_DI A1
#define MAX485_DE A2
#define MAX485_RE A3
#define MAX485_RO A4

SoftwareSerial serial485(/* RX=*/ MAX485_RO, /* TX=*/ MAX485_DI);
#define debugSerial Serial

byte rxCount = 0;
byte rxBuffer[64];
byte txBuffer[64];

/*
 * ===============================================================
 * Ulitlity functions
 * ===============================================================
 */

#ifdef DEBUG
// print debug output on console interface
void debug(debugLevels level, char *sFmt, ...)
{
  if (level > currentDebugLevel) return;  // bypass if level is not high enough
  char acTmp[128];       // place holder for sprintf output
  va_list args;          // args variable to hold the list of parameters
  va_start(args, sFmt);  // mandatory call to initilase args 

  vsprintf(acTmp, sFmt, args);
  debugSerial.print(acTmp);
  // mandatory tidy up
  va_end(args);
  return;
}
#endif

void setup()
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  
  // MAX485 control pins
  pinMode(MAX485_DE, OUTPUT);
  pinMode(MAX485_RE, OUTPUT);
  // Set RX mode
  digitalWrite(MAX485_DE, LOW);
  digitalWrite(MAX485_RE, LOW);
  
  serial485.begin(9600);
  
#ifdef DEBUG   
  debugSerial.begin(9600);
  while (!debugSerial);
  currentDebugLevel = DEBUG_LEVEL_DEFAULT;
  debug(L_INFO, "MAX485 test\n");
#endif
  
  delay(200);
  // check is DTU is in config mode
  if (e32dtu_init()) {
    // in confg mode:
    debug(L_ERROR, "Error: E32-DTU is in configuration mode\n!");
    debug(L_ERROR, "Halt in endless loop\n");
    while(1) {}; // endless loop
  } else {
    debug(L_INFO, "E32-DTU init OK\n");
  }
}

bool e32dtu_init () {
  // Initialise the DTU
  // return:  true=not in config mode (normal operation mode)
  //          false=inititalised, E32-DTU is not ready for normal operation

  // Check if DTU is in config mode 
  txBuffer[0] = 0xC1;
  txBuffer[1] = 0xC1;
  txBuffer[2] = 0xC1;
  write485(3);
  // check for reply
  if ( !receive485(1000,6) ) {
    return false; };    // no reply -> not in config mode -> exit
  // check contents of first byte whcih is fixed
  if ( (rxBuffer[0] != 0xC0) && (rxBuffer[0] != 0xC2) ) {
    return false; };    // incorrect first byte -> not in config mode

  // DTU is in config mode, send configuration
  txBuffer[0] = 0xC0;   // save parameters on power down
  txBuffer[1] = 0x00;   // DTU address High byte
  txBuffer[2] = 0x00;   // DTU address Low byte
  txBuffer[3] = 0x1A;   // 8N1, 9600bps, 1Kbps Airdata
  txBuffer[4] = 0x17;   // Comms channel 433MHz
  txBuffer[5] = 0x44;   // Transparent tx, Push-pull, wireless wakeup 250ms, FEC on, 30dBm TX pwr
  write485(6);
  return false;
}

void write485 (byte txlen) {
  // set TX mode
  digitalWrite(MAX485_DE, HIGH);
  digitalWrite(MAX485_RE, HIGH);
  delay(10);
  // transmitter buffer content
  for (int i = 0; i < txlen; i++) {
    serial485.write(txBuffer[i]);
  }
  // set RX mode
  digitalWrite(MAX485_DE, LOW);
  digitalWrite(MAX485_RE, LOW);
}

bool receive485 (int maxtime, byte len) {
  bool retVal = false;
  unsigned long timeout = millis() + maxtime;
  rxCount=0;
  while (millis() < timeout) {
    if (serial485.available()) {
      rxBuffer[rxCount++] = serial485.read();
    }
    if (rxCount >= len) {
      retVal = true;
      break;
    }
  }
  return retVal;
}

void rx_print() {
  for (int i = 0; i < rxCount; i++) {
    Serial.print("[");
    Serial.print(i);
    Serial.print("] = <");
    Serial.print(rxBuffer[i], HEX);
    Serial.println(">");
  }
}

void loop()
{
  /*
  byte rxByte;
  if (serial485.available()) {
    digitalWrite(LED, HIGH);
    rxByte = serial485.read();
    rxCount++;
    Serial.print("Byte ");
    Serial.print(rxCount);
    Serial.print(" <");
    Serial.print(rxByte, HEX);
    Serial.println(">");
    digitalWrite(LED, LOW);
  }
*/
  digitalWrite(LED, HIGH);
  txBuffer[0] = 'V'; //0xC0;   // save parameters on power down
  txBuffer[1] = 'K'; //0x00;   // DTU address High byte
  txBuffer[2] = '3'; //0x00;   // DTU address Low byte
  txBuffer[3] = 'E'; //0x1A;   // 8N1, 9600bps, 1Kbps Airdata
  txBuffer[4] = 'R'; //0x17;   // Comms channel 433MHz
  txBuffer[5] = 'W'; //0x44;   // Transparent tx, Push-pull, wireless wakeup 250ms, FEC on, 30dBm TX pwr
  txBuffer[6] = 0x0D;
  txBuffer[7] = 0x0A;
  write485(8);
  //delay(150);
  digitalWrite(LED, LOW);
  delay(2000);

}


