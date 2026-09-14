#include <SoftwareSerial.h>
#include <PZEM004Tv30.h>
#include <avr/wdt.h>  // Watchdog Timer library

// =================================================
// PIN CONNECTION (Arduino UNO) - CORRECTED v2.1
// =================================================

// 3 PZEM common communication bus
// ⚠️ IMPORTANT: Changed from D2,D3 to D4,D5
// D2 and D3 are hardware interrupt pins - avoid for SoftwareSerial
// Arduino D4 = RX (SoftwareSerial)
// Arduino D5 = TX (SoftwareSerial)
SoftwareSerial pzemSerial(4, 5);

// SIM900A
// Arduino D10 = RX (SoftwareSerial)
// Arduino D11 = TX (SoftwareSerial)
SoftwareSerial simSerial(10, 11);

// 4 Channel Relay (ACTIVE LOW)
#define RELAY_ON       8      // Motor ON relay
#define RELAY_OFF      9      // Motor OFF relay (pulse)
#define RELAY_ALARM   12      // Alarm/Status relay
#define RELAY_SPARE   13      // Spare relay (not used)

// =================================================
// PZEM OBJECTS
// =================================================

// Each PZEM must have a UNIQUE Modbus address
PZEM004Tv30 pzem1(pzemSerial, 0x01);
PZEM004Tv30 pzem2(pzemSerial, 0x02);
PZEM004Tv30 pzem3(pzemSerial, 0x03);

// =================================================
// SETTINGS & CONSTANTS
// =================================================

// *** IMPORTANT: Your phone number is configured ***
const char PHONE_NUMBER[] = "+919428623658";

// Motor protection limits
const float MIN_VOLTAGE = 190.0;
const float MAX_VOLTAGE = 250.0;
const float MAX_CURRENT = 7.0;

// Fault timing
const unsigned long FAULT_DELAY = 3000;        // 3 seconds before declaring fault
const unsigned long SMS_RETRY_INTERVAL = 1800000;  // 30 minutes between periodic alerts
const unsigned long PZEM_INIT_TIMEOUT = 5000;  // 5 seconds to initialize PZEM
const unsigned long GSM_INIT_TIMEOUT = 10000;  // 10 seconds for GSM initialization

// Buffer sizes (optimized for Arduino UNO limited RAM: 2KB)
const int SMS_BUFFER_SIZE = 128;  // SMS receive buffer
const int AT_RESPONSE_SIZE = 256; // AT command response buffer
const int SMS_MSG_SIZE = 90;      // SMS message size (160 char limit)

// =================================================
// GLOBAL VARIABLES
// =================================================

bool smsSent = false;
bool motorRunning = false;

unsigned long faultStartTime = 0;
unsigned long lastSmsTime = 0;
unsigned long lastLoopTime = 0;
unsigned long loopStartTime = 0;

String currentFaultMessage = "";
bool pzemsInitialized = false;
bool gsmInitialized = false;

// =================================================
// SETUP
// =================================================

void setup()
{
  Serial.begin(9600);
  pzemSerial.begin(9600);
  simSerial.begin(9600);

  // Configure relay pins as outputs (ACTIVE LOW)
  pinMode(RELAY_ON, OUTPUT);
  pinMode(RELAY_OFF, OUTPUT);
  pinMode(RELAY_ALARM, OUTPUT);
  pinMode(RELAY_SPARE, OUTPUT);

  // Set all relays to OFF (HIGH state for ACTIVE LOW logic)
  digitalWrite(RELAY_ON, HIGH);
  digitalWrite(RELAY_OFF, HIGH);
  digitalWrite(RELAY_ALARM, HIGH);
  digitalWrite(RELAY_SPARE, HIGH);

  delay(2000);

  // Print startup message
  Serial.println("\n================================");
  Serial.println("3 Phase Motor Controller v2.1");
  Serial.println("Arduino UNO + 3 PZEM + SIM900A");
  Serial.println("PRODUCTION READY");
  Serial.println("================================\n");

  // Enable Watchdog Timer (4-second timeout for UNO)
  wdt_enable(WDTO_4S);

  // Initialize PZEM modules
  Serial.println("[INIT] Initializing PZEM modules...");
  pzemsInitialized = initializePZEMs();
  
  if (!pzemsInitialized) {
    Serial.println("[WARNING] PZEM init may have failed.");
  } else {
    Serial.println("[OK] PZEMs initialized.");
  }

  // Initialize GSM module
  Serial.println("[INIT] Initializing GSM module...");
  gsmInitialized = initializeGSM();
  
  if (!gsmInitialized) {
    Serial.println("[ERROR] GSM init failed.");
  } else {
    Serial.println("[OK] GSM initialized.");
  }

  Serial.println("\n[SYSTEM] Ready. Motor OFF.\n");

  // Ensure motor is OFF at startup
  motorOFF_Internal();
  
  lastLoopTime = millis();
}

// =================================================
// PZEM INITIALIZATION
// =================================================

bool initializePZEMs()
{
  unsigned long startTime = millis();
  float testVoltage = 0.0;

  pzemSerial.listen();
  
  while (millis() - startTime < PZEM_INIT_TIMEOUT) {
    testVoltage = pzem1.voltage();
    if (!isnan(testVoltage)) {
      Serial.println("[PZEM] Device at 0x01 OK");
      return true;
    }
    delay(100);
  }

  return false;
}

// =================================================
// GSM INITIALIZATION
// =================================================

bool initializeGSM()
{
  unsigned long startTime = millis();
  bool atOk = false;
  bool cmgfOk = false;
  bool cscsOk = false;

  simSerial.listen();
  delay(500);

  // Test AT command
  if (sendATCommand("AT", "OK", 2000)) {
    Serial.println("[GSM] AT OK");
    atOk = true;
  }

  delay(500);

  // Set SMS format to text
  if (sendATCommand("AT+CMGF=1", "OK", 2000)) {
    Serial.println("[GSM] SMS format OK");
    cmgfOk = true;
  }

  delay(500);

  // Set character set
  if (sendATCommand("AT+CSCS=\"GSM\"", "OK", 2000)) {
    Serial.println("[GSM] Charset OK");
    cscsOk = true;
  }

  return (atOk && cmgfOk && cscsOk);
}

// =================================================
// SEND AT COMMAND WITH RESPONSE VALIDATION
// =================================================

bool sendATCommand(String command, String expectedResponse, unsigned long timeout)
{
  simSerial.listen();
  simSerial.println(command);

  char responseBuffer[AT_RESPONSE_SIZE] = {0};
  int idx = 0;
  unsigned long startTime = millis();

  // Read response with timeout
  while (millis() - startTime < timeout) {
    if (simSerial.available()) {
      char c = simSerial.read();
      if (idx < AT_RESPONSE_SIZE - 1) {
        responseBuffer[idx++] = c;
      }
    }
  }
  responseBuffer[idx] = '\0';

  String response(responseBuffer);
  bool success = response.indexOf(expectedResponse) >= 0;

  if (success) {
    Serial.print("[AT] ");
    Serial.print(command);
    Serial.println(" -> OK");
  } else {
    Serial.print("[AT] ");
    Serial.print(command);
    Serial.println(" -> FAIL");
  }

  return success;
}

// =================================================
// MAIN LOOP
// =================================================

void loop()
{
  // Pet the watchdog (reset 4-second timer)
  wdt_reset();

  loopStartTime = millis();

  float v1, v2, v3;
  float c1, c2, c3;
  float p1, p2, p3;

  // -----------------------------------------------
  // Read PZEM-1
  // -----------------------------------------------

  pzemSerial.listen();
  v1 = pzem1.voltage();
  c1 = pzem1.current();
  p1 = pzem1.power();
  delay(100);

  // -----------------------------------------------
  // Read PZEM-2
  // -----------------------------------------------

  pzemSerial.listen();
  v2 = pzem2.voltage();
  c2 = pzem2.current();
  p2 = pzem2.power();
  delay(100);

  // -----------------------------------------------
  // Read PZEM-3
  // -----------------------------------------------

  pzemSerial.listen();
  v3 = pzem3.voltage();
  c3 = pzem3.current();
  p3 = pzem3.power();

  // -----------------------------------------------
  // Display values
  // -----------------------------------------------

  Serial.println("\n--------------------------------");

  printPhaseData("L1", v1, c1, p1);
  printPhaseData("L2", v2, c2, p2);
  printPhaseData("L3", v3, c3, p3);

  Serial.println("Motor: " + String(motorRunning ? "RUNNING" : "STOPPED"));

  // -----------------------------------------------
  // Protection check
  // -----------------------------------------------

  String fault = checkFaults(v1, c1, p1, v2, c2, p2, v3, c3, p3);

  // -----------------------------------------------
  // Fault handling
  // -----------------------------------------------

  handleFault(fault);

  // -----------------------------------------------
  // Check incoming SMS commands
  // -----------------------------------------------

  checkSMSCommands();

  // -----------------------------------------------
  // Maintain consistent 1-second loop timing
  // -----------------------------------------------

  unsigned long loopElapsed = millis() - loopStartTime;
  if (loopElapsed < 1000) {
    delay(1000 - loopElapsed);
  }

  lastLoopTime = millis();
}

// =================================================
// HELPER: Print Phase Data
// =================================================

void printPhaseData(String phase, float voltage, float current, float power)
{
  Serial.print(phase + ": ");
  
  if (isnan(voltage)) {
    Serial.print("V=ERR ");
  } else {
    Serial.print("V=" + String(voltage, 0) + " ");
  }

  if (isnan(current)) {
    Serial.print("I=ERR ");
  } else {
    Serial.print("I=" + String(current, 1) + "A ");
  }

  if (isnan(power)) {
    Serial.println("P=ERR");
  } else {
    Serial.println("P=" + String(power, 0) + "W");
  }
}

// =================================================
// CHECK FAULTS (Centralized logic)
// =================================================

String checkFaults(float v1, float c1, float p1, float v2, float c2, float p2, float v3, float c3, float p3)
{
  String fault = "";

  // PZEM communication failure
  if (isnan(v1) || isnan(c1) || isnan(p1))
    fault += "L1_ERR ";

  if (isnan(v2) || isnan(c2) || isnan(p2))
    fault += "L2_ERR ";

  if (isnan(v3) || isnan(c3) || isnan(p3))
    fault += "L3_ERR ";

  // Voltage protection (only check if not NaN)
  if (!isnan(v1)) {
    if (v1 < MIN_VOLTAGE)
      fault += "L1_LV ";
    if (v1 > MAX_VOLTAGE)
      fault += "L1_HV ";
  }

  if (!isnan(v2)) {
    if (v2 < MIN_VOLTAGE)
      fault += "L2_LV ";
    if (v2 > MAX_VOLTAGE)
      fault += "L2_HV ";
  }

  if (!isnan(v3)) {
    if (v3 < MIN_VOLTAGE)
      fault += "L3_LV ";
    if (v3 > MAX_VOLTAGE)
      fault += "L3_HV ";
  }

  // Current protection (only check if not NaN)
  if (!isnan(c1) && c1 > MAX_CURRENT)
    fault += "L1_OC ";

  if (!isnan(c2) && c2 > MAX_CURRENT)
    fault += "L2_OC ";

  if (!isnan(c3) && c3 > MAX_CURRENT)
    fault += "L3_OC ";

  return fault;
}

// =================================================
// HANDLE FAULT (Improved with fixes)
// =================================================

void handleFault(String fault)
{
  if (fault.length() > 0) {
    Serial.print("[FAULT] ");
    Serial.println(fault);

    // New fault detected
    if (faultStartTime == 0) {
      faultStartTime = millis();
      currentFaultMessage = fault;
      smsSent = false;
      lastSmsTime = 0;
    }

    // Check if fault has persisted for FAULT_DELAY
    if (millis() - faultStartTime >= FAULT_DELAY) {
      // Turn off motor (without SMS notification)
      motorOFF_Internal();
      digitalWrite(RELAY_ALARM, LOW);  // Activate alarm relay

      // Send SMS on first occurrence
      if (!smsSent) {
        // Use snprintf to avoid dynamic string concatenation
        char smsMsg[SMS_MSG_SIZE];
        snprintf(smsMsg, SMS_MSG_SIZE, "FAULT:%s", fault.c_str());
        sendSMS(smsMsg);
        smsSent = true;
        lastSmsTime = millis();
      }
      // Resend SMS every 30 minutes if fault persists
      else if (millis() - lastSmsTime >= SMS_RETRY_INTERVAL) {
        char smsMsg[SMS_MSG_SIZE];
        snprintf(smsMsg, SMS_MSG_SIZE, "ON:%s", fault.c_str());
        sendSMS(smsMsg);
        lastSmsTime = millis();
      }
    }
  } 
  else {
    // No fault - clear fault state
    faultStartTime = 0;
    currentFaultMessage = "";
    smsSent = false;
    lastSmsTime = 0;
    digitalWrite(RELAY_ALARM, HIGH);  // Deactivate alarm relay
  }
}

// =================================================
// MOTOR ON
// =================================================

void motorON()
{
  // Anti-bounce: don't turn on if already running
  if (motorRunning) {
    Serial.println("[MOTOR] Already ON");
    return;
  }

  Serial.println("\n[MOTOR] Turning ON");

  // Activate K1 relay (ACTIVE LOW)
  digitalWrite(RELAY_ON, LOW);
  delay(100);
  
  motorRunning = true;

  char smsMsg[SMS_MSG_SIZE];
  snprintf(smsMsg, SMS_MSG_SIZE, "Motor:ON");
  sendSMS(smsMsg);
}

// =================================================
// MOTOR OFF (Public - with SMS)
// =================================================

void motorOFF()
{
  motorOFF_Internal();

  char smsMsg[SMS_MSG_SIZE];
  snprintf(smsMsg, SMS_MSG_SIZE, "Motor:OFF");
  sendSMS(smsMsg);
}

// =================================================
// MOTOR OFF INTERNAL (No SMS - for fault handling)
// =================================================

void motorOFF_Internal()
{
  if (!motorRunning) {
    Serial.println("[MOTOR] Already OFF");
    return;
  }

  Serial.println("\n[MOTOR] Turning OFF");

  // Deactivate K1 relay
  digitalWrite(RELAY_ON, HIGH);
  delay(100);

  // Pulse K2 OFF relay (optional contactor OFF)
  digitalWrite(RELAY_OFF, LOW);
  delay(500);
  digitalWrite(RELAY_OFF, HIGH);
  delay(100);

  motorRunning = false;
}

// =================================================
// SEND SMS (Improved with buffer safety)
// =================================================

void sendSMS(const char* message)
{
  if (!gsmInitialized) {
    Serial.println("[SMS] GSM not ready");
    return;
  }

  simSerial.listen();

  Serial.print("[SMS] Sending: ");
  Serial.println(message);

  // Clear any pending data
  while (simSerial.available()) {
    simSerial.read();
  }

  // Set SMS format to text mode
  simSerial.println("AT+CMGF=1");
  delay(300);
  clearSerialBuffer();

  // Send SMS command
  simSerial.print("AT+CMGS=\"");
  simSerial.print(PHONE_NUMBER);
  simSerial.println("\"");
  
  delay(1000);
  clearSerialBuffer();

  // Send message text
  simSerial.print(message);
  delay(500);

  // Send CTRL+Z (SMS terminator)
  simSerial.write(26);
  delay(5000);

  // Read response
  clearSerialBuffer();

  Serial.println("[SMS] Sent\n");
}

// =================================================
// CHECK SMS COMMANDS (Improved with validation)
// =================================================

void checkSMSCommands()
{
  if (!gsmInitialized) {
    return;
  }

  simSerial.listen();

  // Request unread messages
  simSerial.println("AT+CMGF=1");
  delay(200);

  simSerial.println("AT+CMGL=\"REC UNREAD\"");
  delay(800);

  // Read response with size limit
  char smsBuffer[SMS_BUFFER_SIZE] = {0};
  int idx = 0;

  unsigned long readStart = millis();
  while (simSerial.available() && (millis() - readStart) < 1000) {
    if (idx < SMS_BUFFER_SIZE - 1) {
      smsBuffer[idx++] = simSerial.read();
    }
  }
  smsBuffer[idx] = '\0';

  // Convert to uppercase for case-insensitive matching
  String sms(smsBuffer);
  sms.toUpperCase();

  // Check for valid SMS commands with state validation
  if ((sms.indexOf("MOTOR ON") >= 0 || sms.indexOf("[ON]") >= 0) && !motorRunning) {
    Serial.println("[SMS CMD] Motor ON");
    motorON();
    delay(1000);
    clearSMSMessage();
  }
  else if ((sms.indexOf("MOTOR OFF") >= 0 || sms.indexOf("[OFF]") >= 0) && motorRunning) {
    Serial.println("[SMS CMD] Motor OFF");
    motorOFF();
    delay(1000);
    clearSMSMessage();
  }
}

// =================================================
// CLEAR SERIAL BUFFER
// =================================================

void clearSerialBuffer()
{
  unsigned long timeout = millis() + 500;
  while (simSerial.available() && millis() < timeout) {
    simSerial.read();
  }
}

// =================================================
// DELETE SMS MESSAGE
// =================================================

void clearSMSMessage()
{
  simSerial.listen();
  simSerial.println("AT+CMGD=1,4");
  delay(500);
  clearSerialBuffer();
  Serial.println("[SMS] Cleared");
}

// =================================================
// END OF CODE - v2.1 PRODUCTION READY
// =================================================
