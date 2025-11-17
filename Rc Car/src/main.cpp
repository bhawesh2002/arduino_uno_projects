#include <Arduino.h>
#include <AFMotor.h>
// Using the hardware UART for HC-05
#define BTSerial Serial
// Define motor objects using AFMotor library
AF_DCMotor motor1(1, MOTOR12_1KHZ); // Motor 1
AF_DCMotor motor2(2, MOTOR12_1KHZ); // Motor 2
AF_DCMotor motor3(3, MOTOR34_1KHZ); // Motor 3
AF_DCMotor motor4(4, MOTOR34_1KHZ); // Motor 4
// Safety timeout (ms) — stops motors if no command received
unsigned long lastBTTime = 0;
const unsigned long BT_TIMEOUT = 1500; // 1.5 seconds

// Function prototypes
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMotors();

void setup()
{
  BTSerial.begin(9600);
  Serial.begin(9600);

  // Initialize motors to stop
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);

  stopMotors();

  Serial.println("RC Car Ready");
}

void loop()
{

  // Check for Bluetooth commands
  if (BTSerial.available())
  {
    char c = BTSerial.read();
    Serial.print("BT Command: ");
    Serial.println(c);

    lastBTTime = millis(); // update last command time

    switch (c)
    {
    case 'F':
    case 'f':
      moveForward();
      break;

    case 'B':
    case 'b':
      moveBackward();
      break;

    case 'L':
    case 'l':
      turnLeft();
      break;

    case 'R':
    case 'r':
      turnRight();
      break;

    case 'S':
    case 's':
      stopMotors();
      break;

    default:
      break;
    }
  }

  if (millis() - lastBTTime > BT_TIMEOUT)
  {
    stopMotors();
  }
}

// =======================
// Motor control functions
// =======================

void moveForward()
{
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void moveBackward()
{
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void turnLeft()
{
  // left motors backward, right motors forward
  motor1.run(FORWARD); // Motor 1 moves backward
  motor2.run(FORWARD); // Motor 2 moves forward
  motor3.run(RELEASE); // Motor 3 moves forward
  motor4.run(RELEASE); // Motor 4 moves backward
}

void turnRight()
{
  // left motors forward, right motors backward
  motor1.run(RELEASE);  // Motor 1 moves backward
  motor2.run(RELEASE);  // Motor 2 moves forward
  motor3.run(BACKWARD); // Motor 3 moves forward
  motor4.run(BACKWARD); // Motor 4 moves backward
}

void stopMotors()
{
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
