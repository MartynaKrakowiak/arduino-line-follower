#include <Arduino.h>
#include <NewPing.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// Motor Control Pins
#define ENA 3 // Enable motors on the right side of the robot
#define ENB 11 // Enable motors on the left side of the robot
#define IN1 6 // Control the direction of rotation of motors on the right side
#define IN2 7 // Control the direction of rotation of motors on the right side
#define IN3 8 // Control the direction of rotation of motors on the left side
#define IN4 9 // Control the direction of rotation of motors on the left side
#define SPEED 200
#define MAX_DISTANCE 15
#define TRIGGER 2
#define ECHO 4
#define RIGHT_IR_SENSOR 13
#define MIDDLE_IR_SENSOR 12
#define LEFT_IR_SENSOR 5

int bt_data;
int mode;
long duration;
int distance;
int dis;
int left_distance;
int right_distance;
boolean object;
Servo servo;
int tim;
SoftwareSerial bluetooth(0, 1); // Bluetooth module communication pins (TX, RX)
int state;
int bluetooth_data;

void setup() {
  pinMode(RIGHT_IR_SENSOR, INPUT);
  pinMode(MIDDLE_IR_SENSOR, INPUT);
  pinMode(LEFT_IR_SENSOR, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  servo.attach(10);
  servo.write(90);
  bluetooth.begin(9600);
  Serial.begin(9600);
}

// Function to get the distance from the ultrasonic sensor
int get_distance() {
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  // Measure the duration of the ECHO pulse
  duration = pulseIn(ECHO, HIGH);
  // Calculate the distance using the formula presented in section 2.1.2
  distance = duration / 58;
  return distance;
}

// Function to look to the left using the servo
int look_left() {
  servo.write(150);
  delay(500);
  left_distance = get_distance();
  delay(100);
  servo.write(90);
  Serial.println("Left distance: ");
  Serial.println(left_distance);
  return left_distance;
  delay(100);
}

// Function to look to the right using the servo
int look_right() {
  servo.write(30);
  delay(500);
  right_distance = get_distance();
  delay(100);
  servo.write(90);
  Serial.println("Right distance: ");
  Serial.println(right_distance);
  return left_distance;
  delay(100);
}

// Function to perform a turn maneuver based on obstacle direction
void turn() {
  if (object == false) {
    turn_left(120);
    delay(800);
    go_forward(120);
    delay(800);
    turn_right(120);
    delay(800);

    // If the robot encounters a black line, return to the main program loop
    if (digitalRead(RIGHT_IR_SENSOR) == 1) {
      loop();
    }
    // If the robot does not encounter a black line, go straight
    else {
      go_forward(140);
    }
  } else {
    turn_right(120);
    delay(800);
    go_forward(120);
    delay(800);
    turn_left(120);
    delay(800);

    if (digitalRead(LEFT_IR_SENSOR) == 1) {
      loop();
    } else {
      go_forward(140);
    }
  }
}

// Function to avoid obstacles using the ultrasonic sensor
void avoid_obstacle() {
  // Measure the distance
  distance = get_distance();
  if (distance <= 15) {
    Stop();
    look_left();
    look_right();
    delay(100);

    // If the distance from the obstacle on the left side is greater than on the right,
    // set the boolean variable 'object' to true;
    // the 'turn()' function will command to perform the obstacle avoidance maneuver from the left side
    if (right_distance <= left_distance) {
      object = true;
      turn();
    } else {
      object = false;
      turn();
    }

    delay(100);
  }
}

void loop() {
  if (Serial.available() > 0) {
    bluetooth_data = Serial.read();
    Serial.println(bluetooth_data);
  }

  if (bluetooth_data == 7) {
    state = 0;
    Stop();
  } else if (bluetooth_data == 6) {
    state = 1;
  }

  if (state == 0) {
    if (bluetooth_data == 1) {
      go_forward(150);
    } else if (bluetooth_data == 2) {
      go_back(150);
    } else if (bluetooth_data == 3) {
      turn_left(150);
    } else if (bluetooth_data == 4) {
      turn_right(150);
    } else if (bluetooth_data == 5) {
      Stop();
    }
  }

  if (state == 1) {
    if ((digitalRead(LEFT_IR_SENSOR) == 0) && (digitalRead(MIDDLE_IR_SENSOR) == 1) && (digitalRead(RIGHT_IR_SENSOR) == 0)) {
      avoid_obstacle();
      go_forward(150);
    } else if ((digitalRead(LEFT_IR_SENSOR) == 1) && (digitalRead(MIDDLE_IR_SENSOR) == 1) && (digitalRead(RIGHT_IR_SENSOR) == 0)) {
      avoid_obstacle();
      turn_left(150);
    } else if ((digitalRead(LEFT_IR_SENSOR) == 1) && (digitalRead(MIDDLE_IR_SENSOR) == 0) && (digitalRead(RIGHT_IR_SENSOR) == 0)) {
      avoid_obstacle();
      turn_left(150);
    } else if ((digitalRead(LEFT_IR_SENSOR) == 0) && (digitalRead(MIDDLE_IR_SENSOR) == 1) && (digitalRead(RIGHT_IR_SENSOR) == 1)) {
      avoid_obstacle();
      turn_right(150);
    } else if ((digitalRead(LEFT_IR_SENSOR) == 0) && (digitalRead(MIDDLE_IR_SENSOR) == 0) && (digitalRead(RIGHT_IR_SENSOR) == 1)) {
      avoid_obstacle();
      turn_right(150);
    } else if ((digitalRead(LEFT_IR_SENSOR) == 1) && (digitalRead(MIDDLE_IR_SENSOR) == 1) && (digitalRead(RIGHT_IR_SENSOR) == 1)) {
      Stop();
    }
  }
}

void go_back(int val) {
  analogWrite(ENA, val);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENB, val);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void go_forward(int val) {
  analogWrite(ENA, val);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENB, val);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turn_left(int val) {
  analogWrite(ENA, val);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENB, val);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turn_right(int val) {
  analogWrite(ENA, val);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENB, val);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void Stop() {
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
