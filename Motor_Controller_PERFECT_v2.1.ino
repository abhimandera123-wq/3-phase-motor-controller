#include <SoftwareSerial.h>
#include <PZEM004Tv30.h>
#include <avr/wdt.h>

// =================================================
// PIN CONFIGURATION (Arduino UNO)
// =================================================

// PZEM Serial (D4=RX, D5=TX) - IMPORTANT: NOT D2,D3
SoftwareSerial pzemSerial(4, 5);

// GSM Serial (D10=RX, D11=TX)
SoftwareSerial simSerial(10, 11);

// Relay pins
#define RELAY_ON       8
#define RELAY_OFF      9
#define RELAY_ALARM   12
#define RELAY_SPARE   13

// =================================================
// PZEM INSTANCES (Modbus addresses 0x01, 0x02, 0x03)
// =================================================

PZEM004Tv30 pzem1(pzemSerial, 0x01);
PZEM004Tv30 pzem2(pzemSerial, 0x02);
PZEM004Tv30 pzem3(pzemSerial, 0x03);

// =================================================
// CONFIGURATION
// =================================================

const char PHONE_NUMBER[] = "+919428623658";

const float MIN_VOLTAGE = 190.0;
const float MAX_VOLTAGE = 250.0;
const float MAX_CURRENT = 7.0;

const unsigned long FAULT_DELAY = 3000;
const unsigned long SMS_RETRY_INTERVAL = 1800000;
const unsigned long PZEM_INIT_TIMEOUT = 5000;
const unsigned long GSM_INIT_TIMEOUT = 10000;

const int SMS_BUFFER_SIZE = 128;
const int AT_RESPONSE_SIZE = 256;
const int SMS_MSG_SIZE = 90;

// =================================================
// GLOBAL VARIABLES
// =================================================

bool smsSent = false;
bool motorRunning = false;
unsigned long faultStartTime = 0;
unsigned long lastSmsTime = 0;
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

  pinMode(RELAY_ON, OUTPUT);
  pinMode(RELAY_OFF, OUTPUT);
  pinMode(RELAY_ALARM, OUTPUT);
  pinMode(RELAY_SPARE, OUTPUT);

  digitalWrite(RELAY_ON, HIGH);
  digitalWrite(RELAY_OFF, HIGH);
  digitalWrite(RELAY_ALARM, HIGH);
  digitalWrite(RELAY_SPARE, HIGH);

  delay(2000);

  Serial.println("\n================================");
  Serial.println("3-Phase Motor Controller v2.1");
  Serial.println("Arduino UNO + PZEM + SIM900A");
  Serial.println("================================\n");

  wdt_enable(WDTO_4S);

  Serial.println("[INIT] PZEM initialization...");
  pzemsInitialized = initializePZEMs();
  Serial.println(pzemsInitialized ? "[OK] PZEM ready" : "[WARN] PZEM check failed");

  Serial.println("[INIT] GSM initialization...");
  gsmInitialized = initializeGSM();
  Serial.println(gsmInitialized ? "[OK] GSM ready" : "[ERROR] GSM failed");

  Serial.println("\n[SYSTEM] Ready\n");
  motorOFF_Internal();
}

// =================================================
// PZEM INIT
// =================================================

bool initializePZEMs()
{
  unsigned long startTime = millis();
  pzemSerial.listen();
  
  while (millis() - startTime < PZEM_INIT_TIMEOUT) {
    if (!isnan(pzem1.voltage())) {
      Serial.println("[PZEM] Device 0x01 detected");
      return true;
    }
    delay(100);
  }
  return false;
}

// =================================================
// GSM INIT
// =================================================

bool initializeGSM()
{
  simSerial.listen();
  delay(500);

  bool atOk = sendATCommand("AT", "OK", 2000);
  delay(300);
  bool cmgfOk = sendATCommand("AT+CMGF=1", "OK", 2000);
  delay(300);
  bool cscsOk = sendATCommand("AT+CSCS=\"GSM\"", "OK", 2000);

  return (atOk && cmgfOk && cscsOk);
}

// =================================================
// AT COMMAND SEND
// =================================================

bool sendATCommand(String command, String expectedResponse, unsigned long timeout)
{
  simSerial.listen();
  simSerial.println(command);

  char responseBuffer[AT_RESPONSE_SIZE] = {0};
  int idx = 0;
  unsigned long startTime = millis();

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
    Serial.println("[AT] " + command + " OK");
  } else {
    Serial.println("[AT] " + command + " FAILED");
  }

  return success;
}

// =================================================
// MAIN LOOP
// =================================================

void loop()
{
  wdt_reset();
  loopStartTime = millis();

  float v1, v2, v3, c1, c2, c3, p1, p2, p3;

  pzemSerial.listen();
  v1 = pzem1.voltage();
  c1 = pzem1.current();
  p1 = pzem1.power();
  delay(100);

  pzemSerial.listen();
  v2 = pzem2.voltage();
  c2 = pzem2.current();
  p2 = pzem2.power();
  delay(100);

  pzemSerial.listen();
  v3 = pzem3.voltage();
  c3 = pzem3.current();
  p3 = pzem3.power();

  Serial.println("\n--------------------------------");
  printPhaseData("L1", v1, c1, p1);
  printPhaseData("L2", v2, c2, p2);
  printPhaseData("L3", v3, c3, p3);
  Serial.println("Status: " + String(motorRunning ? "RUNNING" : "STOPPED"));

  String fault = checkFaults(v1, c1, p1, v2, c2, p2, v3, c3, p3);
  handleFault(fault);
  checkSMSCommands();

  unsigned long loopElapsed = millis() - loopStartTime;
  if (loopElapsed < 1000) {
    delay(1000 - loopElapsed);
  }
}

// =================================================
// PRINT PHASE DATA
// =================================================

void printPhaseData(String phase, float voltage, float current, float power)
{
  Serial.print(phase + ": ");
  Serial.print(isnan(voltage) ? "V=ERR " : "V=" + String(voltage, 0) + " ");
  Serial.print(isnan(current) ? "I=ERR " : "I=" + String(current, 1) + "A ");
  Serial.println(isnan(power) ? "P=ERR" : "P=" + String(power, 0) + "W");
}

// =================================================
// CHECK FAULTS
// =================================================

String checkFaults(float v1, float c1, float p1, float v2, float c2, float p2, float v3, float c3, float p3)
{
  String fault = "";

  if (isnan(v1) || isnan(c1) || isnan(p1)) fault += "L1_ERR ";
  if (isnan(v2) || isnan(c2) || isnan(p2)) fault += "L2_ERR ";
  if (isnan(v3) || isnan(c3) || isnan(p3)) fault += "L3_ERR ";

  if (!isnan(v1)) {
    if (v1 < MIN_VOLTAGE) fault += "L1_LV ";
    if (v1 > MAX_VOLTAGE) fault += "L1_HV ";
  }
  if (!isnan(v2)) {
    if (v2 < MIN_VOLTAGE) fault += "L2_LV ";
    if (v2 > MAX_VOLTAGE) fault += "L2_HV ";
  }
  if (!isnan(v3)) {
    if (v3 < MIN_VOLTAGE) fault += "L3_LV ";
    if (v3 > MAX_VOLTAGE) fault += "L3_HV ";
  }

  if (!isnan(c1) && c1 > MAX_CURRENT) fault += "L1_OC ";
  if (!isnan(c2) && c2 > MAX_CURRENT) fault += "L2_OC ";
  if (!isnan(c3) && c3 > MAX_CURRENT) fault += "L3_OC ";

  return fault;
}

// =================================================
// HANDLE FAULT
// =================================================

void handleFault(String fault)
{
  if (fault.length() > 0) {
    Serial.println("[FAULT] " + fault);

    if (faultStartTime == 0) {
      faultStartTime = millis();
      currentFaultMessage = fault;
      smsSent = false;
      lastSmsTime = 0;
    }

    if (millis() - faultStartTime >= FAULT_DELAY) {
      motorOFF_Internal();
      digitalWrite(RELAY_ALARM, LOW);

      if (!smsSent) {
        char smsMsg[SMS_MSG_SIZE];
        snprintf(smsMsg, SMS_MSG_SIZE, "FAULT:%s", fault.c_str());
        sendSMS(smsMsg);
        smsSent = true;
        lastSmsTime = millis();
      }
      else if (millis() - lastSmsTime >= SMS_RETRY_INTERVAL) {
        char smsMsg[SMS_MSG_SIZE];
        snprintf(smsMsg, SMS_MSG_SIZE, "ONGOING:%s", fault.c_str());
        sendSMS(smsMsg);
        lastSmsTime = millis();
      }
    }
  }
  else {
    faultStartTime = 0;
    currentFaultMessage = "";
    smsSent = false;
    lastSmsTime = 0;
    digitalWrite(RELAY_ALARM, HIGH);
  }
}

// =================================================
// MOTOR ON
// =================================================

void motorON()
{
  if (motorRunning) return;

  Serial.println("\n[MOTOR] ON");
  digitalWrite(RELAY_ON, LOW);
  delay(100);
  motorRunning = true;

  char smsMsg[SMS_MSG_SIZE];
  snprintf(smsMsg, SMS_MSG_SIZE, "Motor:ON");
  sendSMS(smsMsg);
}

// =================================================
// MOTOR OFF (Public)
// =================================================

void motorOFF()
{
  motorOFF_Internal();

  char smsMsg[SMS_MSG_SIZE];
  snprintf(smsMsg, SMS_MSG_SIZE, "Motor:OFF");
  sendSMS(smsMsg);
}

// =================================================
// MOTOR OFF INTERNAL (No SMS)
// =================================================

void motorOFF_Internal()
{
  if (!motorRunning) return;

  Serial.println("\n[MOTOR] OFF");
  digitalWrite(RELAY_ON, HIGH);
  delay(100);
  digitalWrite(RELAY_OFF, LOW);
  delay(500);
  digitalWrite(RELAY_OFF, HIGH);
  delay(100);
  motorRunning = false;
}

// =================================================
// SEND SMS
// =================================================

void sendSMS(const char* message)
{
  if (!gsmInitialized) return;

  simSerial.listen();

  Serial.print("[SMS] Sending: ");
  Serial.println(message);

  while (simSerial.available()) simSerial.read();

  simSerial.println("AT+CMGF=1");
  delay(300);
  clearSerialBuffer();

  simSerial.print("AT+CMGS=\"");
  simSerial.print(PHONE_NUMBER);
  simSerial.println("\"");
  delay(1000);
  clearSerialBuffer();

  simSerial.print(message);
  delay(500);
  simSerial.write(26);
  delay(5000);
  clearSerialBuffer();

  Serial.println("[SMS] Sent\n");
}

// =================================================
// CHECK SMS COMMANDS
// =================================================

void checkSMSCommands()
{
  if (!gsmInitialized) return;

  simSerial.listen();

  simSerial.println("AT+CMGF=1");
  delay(200);
  simSerial.println("AT+CMGL=\"REC UNREAD\"");
  delay(800);

  char smsBuffer[SMS_BUFFER_SIZE] = {0};
  int idx = 0;

  unsigned long readStart = millis();
  while (simSerial.available() && (millis() - readStart) < 1000) {
    if (idx < SMS_BUFFER_SIZE - 1) {
      smsBuffer[idx++] = simSerial.read();
    }
  }
  smsBuffer[idx] = '\0';

  String sms(smsBuffer);
  sms.toUpperCase();

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
// DELETE SMS
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
// END
// =================================================
