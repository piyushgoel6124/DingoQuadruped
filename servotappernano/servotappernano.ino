#include <Servo.h>

// IDENTIFICATION CONFIGURATION
// Compile and upload with "x1" to the Left Nano, and "x2" to the Right Nano.
const String BOARD_ID = "x1"; 

Servo servos[6];

const byte servoPins[6]  = {3, 4, 5, 6, 7, 8};
const byte analogPins[6] = {A0, A1, A2, A3, A6, A7};

String rxBuffer = "";

unsigned long lastReport = 0;
const unsigned long reportInterval = 100; // ms

bool servoAttached[6];

// Watchdog and active status variables
bool active = false;
unsigned long lastCommandTime = 0;
const unsigned long watchdogTimeout = 5000; // 5 seconds

void setup() {
  Serial.begin(115200);

  // Connect AREF to same 3.3V rail as servo pots
  analogReference(EXTERNAL);

  for (int i = 0; i < 6; i++) {
    servoAttached[i] = false;
  }
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      processCommand(rxBuffer);
      rxBuffer = "";
    }
    else if (c != '\r') {
      rxBuffer += c;
    }
  }

  // Watchdog: detach servos and stop sending analogs if no serial communications
  if (active && (millis() - lastCommandTime >= watchdogTimeout)) {
    active = false;
    for (int i = 0; i < 6; i++) {
      if (servoAttached[i]) {
        servos[i].detach();
        servoAttached[i] = false;
      }
    }
  }

  if (active && (millis() - lastReport >= reportInterval)) {
    lastReport = millis();
    sendAnalogs();
  }
}

void processCommand(String cmd) {
  // Any incoming command activates telemetry/servo control
  active = true;
  lastCommandTime = millis();

  if (cmd == "START") {
    // Just reset watchdog and enable active reporting
    return;
  }

  if (cmd == "STOP") {
    active = false;
    for (int i = 0; i < 6; i++) {
      if (servoAttached[i]) {
        servos[i].detach();
        servoAttached[i] = false;
      }
    }
    return;
  }

  if (cmd == "ID") {
    Serial.println("ID:" + BOARD_ID);
    return;
  }

  int v[6];

  int count = sscanf(
    cmd.c_str(),
    "%d,%d,%d,%d,%d,%d",
    &v[0], &v[1], &v[2],
    &v[3], &v[4], &v[5]
  );

  if (count != 6) return;

  for (int i = 0; i < 6; i++) {

    // 404 = detach servo
    if (v[i] == 404) {
      if (servoAttached[i]) {
        servos[i].detach();
        servoAttached[i] = false;
      }
      continue;
    }

    // 200 = reattach servo
    if (v[i] == 200) {
      if (!servoAttached[i]) {
        servos[i].attach(servoPins[i]);
        servoAttached[i] = true;
      }
      continue;
    }

    // Normal angle command
    if (v[i] >= 0 && v[i] <= 180) {

      if (!servoAttached[i]) {
        servos[i].attach(servoPins[i]);
        servoAttached[i] = true;
      }

      servos[i].write(v[i]);
    }
  }
}

void sendAnalogs() {

  for (int i = 0; i < 6; i++) {
    Serial.print(analogRead(analogPins[i]));

    if (i < 5)
      Serial.print(',');
  }

  Serial.println();
}