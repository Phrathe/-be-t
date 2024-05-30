#include <TimerOne.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

unsigned int counter = 0;
int irSensorPin = 2; // IR sensor input pin

const byte Row = 4;
const byte Cols = 3;

char hx[Row][Cols] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

byte rowPins[Row] = {7, 12, 11, 9};
byte colPins[Cols] = {8, 6, 10};

Keypad customKeypad = Keypad(makeKeymap(hx), rowPins, colPins, Row, Cols);

LiquidCrystal_I2C lcd(0x27, 16, 2);
int setpoint = 0;
int rotation = 0; // Initialize rotation variable
int prevRPM = 0; // Variable to store previous RPM

const int enaPin = 13; // PWM pin for motor speed control
const int in1Pin = 3;  // Motor control pin 1
const int in2Pin = 4;  // Motor control pin 2

unsigned long previousMillis = 0; // Variable to store previous time

bool motorControlEnabled = true; // Flag to enable/disable motor control

void timerIsr() {
  Timer1.detachInterrupt();  // stop the timer
  int currentRPM = (counter / 20) * 60; // Calculate RPM based on 20 pulses per rotation
  if (currentRPM != prevRPM) {
    prevRPM = currentRPM;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Actual RPM: ");
    lcd.print(currentRPM);
  }
  counter = 0;   // reset counter to 0
  Timer1.attachInterrupt(timerIsr);  // enable timer
}

void setup() {
  Serial.begin(19200);

  pinMode(irSensorPin, INPUT); // IR sensor pin setup
  pinMode(enaPin, OUTPUT);     // Motor control pins as outputs
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  digitalWrite(enaPin, LOW);    // Initialize motor pins

  Timer1.initialize(1000000);  // setting timer for 1 second (1000000 microseconds)
  attachInterrupt(digitalPinToInterrupt(irSensorPin), docount, RISING);  // increase counter when speed sensor goes high
  Timer1.attachInterrupt(timerIsr);

  lcd.init();                   // Initialize LCD
  lcd.backlight();
  
  // Ask for motor control option on startup
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Motor Control?");
  lcd.setCursor(0, 1);
  lcd.print("1: Yes   2: No");
  char choice = waitForKey('1', '2');
  motorControlEnabled = (choice == '1');
}

void loop() {
  if (motorControlEnabled) {
    enableMotorControl(); // Check for keypad input and update motor control
  } else {
    // Display constant RPM without motor control
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Constant RPM:");
    lcd.setCursor(0, 1);
    lcd.print(rotation); // Display current RPM without motor control
  }
}

void enableMotorControl() {
  char customKey = customKeypad.getKey();

  if (customKey) {
    int percentage = 0; // Initialize percentage variable

    switch (customKey) {
      case '0':
        percentage = 0;   // Minimum speed
        break;
      case '1':
        percentage = 25;  // Speed level 1 (25%)
        break;
      case '2':
        percentage = 50;  // Speed level 2 (50%)
        break;
      case '3':
        percentage = 75;  // Speed level 3 (75%)
        break;
      case '4':
        percentage = 100; // Maximum speed (100%)
        break;
      default:
        break;
    }

    setpoint = map(percentage, 0, 100, 0, 255); // Map percentage to PWM range (0-255)
    analogWrite(enaPin, setpoint); // Set motor speed based on percentage input
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Setpoint: ");
    lcd.print(percentage); // Display percentage value on LCD
    lcd.print("%"); // Display percentage symbol

    Serial.print("Setpoint: ");
    Serial.println(percentage);
  }
}

void docount() {
  if (millis() - previousMillis >= 1000) { // Check if 1 second has elapsed
    rotation = (counter / 20) * 60; // Calculate RPM based on 20 pulses per rotation
    counter = 0; // Reset the counter
    previousMillis = millis(); // Update the previousMillis variable
  }
  counter++; // increase counter by 1
}

char waitForKey(char option1, char option2) {
  char key;
  do {
    key = customKeypad.getKey();
  } while (key != option1 && key != option2);
  return key;
}
