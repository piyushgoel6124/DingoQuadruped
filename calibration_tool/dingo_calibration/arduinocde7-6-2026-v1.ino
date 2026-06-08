#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca9685;

#define NUM_SERVOS 16

#define SERVO_MIN 110
#define SERVO_MAX 490

int currentAngle[NUM_SERVOS];
int targetAngle[NUM_SERVOS];

unsigned long lastUpdate = 0;

int angleToPulse(int angle)
{
    angle = constrain(angle, 0, 180);
    return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void writeServo(int ch, int angle)
{
    pca9685.setPWM(ch, 0, angleToPulse(angle));
}

void processPacket(String line)
{
    int count = 0;
    int start = 0;

    while (count < NUM_SERVOS)
    {
        int comma = line.indexOf(',', start);

        String token;

        if (comma == -1)
            token = line.substring(start);
        else
            token = line.substring(start, comma);

        token.trim();

        if (token.length() == 0)
            return;

        targetAngle[count++] =
            constrain(token.toInt(), 0, 180);

        if (comma == -1)
            break;

        start = comma + 1;
    }

    if (count != NUM_SERVOS)
        return;
}

void setup()
{
    Serial.begin(115200);

    pca9685.begin();
    pca9685.setPWMFreq(50);

    delay(500);

    for (int i = 0; i < NUM_SERVOS; i++)
    {
        currentAngle[i] = 90;
        targetAngle[i] = 90;

        writeServo(i, 90);
    }
}

void loop()
{
    static String rx = "";

    while (Serial.available())
    {
        char c = Serial.read();

        if (c == '\n')
        {
            if (rx.length() > 0)
            {
                processPacket(rx);
                rx = "";
            }
        }
        else if (c != '\r')
        {
            rx += c;
        }
    }

    if (millis() - lastUpdate >= 20)
    {
        lastUpdate = millis();

        for (int i = 0; i < NUM_SERVOS; i++)
        {
            int error = targetAngle[i] - currentAngle[i];

            if (error == 0)
                continue;

            int step;

            if (abs(error) > 60)
                step = 8;
            else if (abs(error) > 30)
                step = 5;
            else if (abs(error) > 15)
                step = 3;
            else if (abs(error) > 5)
                step = 2;
            else
                step = 1;

            if (error > 0)
            {
                currentAngle[i] += step;

                if (currentAngle[i] > targetAngle[i])
                    currentAngle[i] = targetAngle[i];
            }
            else
            {
                currentAngle[i] -= step;

                if (currentAngle[i] < targetAngle[i])
                    currentAngle[i] = targetAngle[i];
            }

            writeServo(i, currentAngle[i]);
        }
    }
}