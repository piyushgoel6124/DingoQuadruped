/*
 * RoboDog Quadruped - Arduino Uno PCA9685 Pure Actuator Interface
 *
 * This sketch runs on the Arduino Uno and acts as a direct, non-blocking hardware output.
 * It has no baseline offset storage, logic profiles, or automatic initial poses on startup.
 * All joints remain completely unactuated (limp) on boot to prevent sudden jerking.
 * The entire calculation brain (offsets, inversions, kinematics) runs on the host PC.
 *
 * Hardware Connections:
 *   Arduino Uno A4 (SDA) -> PCA9685 SDA
 *   Arduino Uno A5 (SCL) -> PCA9685 SCL
 *   Arduino Uno 5V       -> PCA9685 VCC (logic)
 *   Arduino Uno GND      -> PCA9685 GND
 *   External Power (~7V) -> PCA9685 Green Terminal Block V+ and V-
 *
 * Serial Command Protocol (115200 baud):
 *   M <joint_idx> <raw_angle>   : Command a specific joint (0-11) to a raw physical angle (0.0-180.0)
 *   A <a0> <a1> ... <a11>       : Command all 12 joints simultaneously to raw physical angles
 *   R                           : Relax all 16 channels immediately (limp state)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Servo PWM pulse limits (microseconds)
#define USMIN 500  // Standard minimum pulse width (0 degrees)
#define USMAX 2500 // Standard maximum pulse width (180 degrees)
#define SERVO_FREQ 50 // Update rate for analog servos (~50Hz)

// Instantiate the PCA9685 driver (Default I2C address: 0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo Pin Mappings matching indices 0 to 11
const int servoPins[12] = {14, 13, 12, 10, 9, 8, 2, 1, 0, 6, 5, 4};

void moveServoRaw(int jointId, float rawAngle) {
  if (jointId < 0 || jointId >= 12) return;
  
  // Constrain angle to physical servo limits to prevent binding
  float constrainedAngle = constrain(rawAngle, 0.0, 180.0);
  
  // Convert angle to pulse width in microseconds
  int pulseUs = map(constrainedAngle, 0, 180, USMIN, USMAX);
  
  // Write directly to the PCA9685 mapped pin channel
  pwm.writeMicroseconds(servoPins[jointId], pulseUs);
}

void relaxAllMotors() {
  for (int i = 0; i < 16; i++) {
    pwm.setPWM(i, 0, 4096); // 4096 turns off PWM pulse completely (limp state)
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  
  // CRITICAL: Do NOT actuate any servos on bootup.
  // Leave all channels in a completely unactuated/limp state to prevent sudden jerking.
  relaxAllMotors();
  
  Serial.println("INFO RoboDog Pure Hardware Interface Online");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;
    
    char cmd = input.charAt(0);
    
    if (cmd == 'M' || cmd == 'm') {
      // Command format: M <joint_idx> <raw_angle>
      int space1 = input.indexOf(' ');
      int space2 = input.indexOf(' ', space1 + 1);
      if (space1 != -1 && space2 != -1) {
        int jointId = input.substring(space1 + 1, space2).toInt();
        float rawAngle = input.substring(space2 + 1).toFloat();
        if (jointId >= 0 && jointId < 12) {
          moveServoRaw(jointId, rawAngle);
          Serial.print("ACK M ");
          Serial.print(jointId);
          Serial.print(" ");
          Serial.println(rawAngle);
        } else {
          Serial.println("ERR Invalid joint ID");
        }
      } else {
        Serial.println("ERR Format: M <joint_idx> <raw_angle>");
      }
    } 
    else if (cmd == 'A' || cmd == 'a') {
      // Command format: A <a0> <a1> ... <a11>
      int nextSpace = input.indexOf(' ');
      bool valid = true;
      float targetAngles[12];
      
      for (int i = 0; i < 12; i++) {
        if (nextSpace == -1) {
          valid = false;
          break;
        }
        int prevSpace = nextSpace;
        nextSpace = input.indexOf(' ', prevSpace + 1);
        String valStr = (nextSpace == -1) ? input.substring(prevSpace + 1) : input.substring(prevSpace + 1, nextSpace);
        targetAngles[i] = valStr.toFloat();
      }
      
      if (valid) {
        for (int i = 0; i < 12; i++) {
          moveServoRaw(i, targetAngles[i]);
        }
        Serial.println("ACK A");
      } else {
        Serial.println("ERR Format: A <a0> <a1> ... <a11>");
      }
    }
    else if (cmd == 'R' || cmd == 'r') {
      // Command format: R (Relax all motors immediately)
      relaxAllMotors();
      Serial.println("ACK R");
    }
    else {
      Serial.println("ERR Unknown command");
    }
  }
}
