#include <Servo.h>

Servo servos[6];

const byte servoPins[6]  = {3, 4, 5, 6, 7, 8};
const byte analogPins[6] = {A0, A1, A2, A3, A6, A7};

String rxBuffer = "";

unsigned long lastReport = 0;
const unsigned long reportInterval = 100; // ms

bool servoAttached[6];

void setup() {
  Serial.begin(115200);

  // Connect AREF to same 3.3V rail as servo pots
  analogReference(EXTERNAL);

  for (int i = 0; i < 6; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(90);
    servoAttached[i] = true;
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

  if (millis() - lastReport >= reportInterval) {
    lastReport = millis();
    sendAnalogs();
  }
}

void processCommand(String cmd) {

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