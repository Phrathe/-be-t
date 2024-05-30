#include <TimerOne.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

int irSensorPin = 2; // IR sensor input pin
const byte Row = 4;
const byte Cols = 3;
volatile unsigned long countPulse = 0;
double Kp, Ki, Kd;
double dt, last_time;
double integral, previous_error, signal_Out = 0;
double error;
int sp = 0;
double filtered_signal_Out = 0;
const double alpha = 0.1; // IIR filter coefficient

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

const int enaPin = 13; // PWM pin for motor speed control
const int in1Pin = 3;  // Motor control pin 1
const int in2Pin = 4;  // Motor control pin 2

void setup() {
  Serial.begin(9600);

  Kp = -0.076;
  Ki = 0;
  Kd = 0;
  dt = 0.1; // Initial dt value
  
  pinMode(irSensorPin, INPUT); // IR sensor pin setup
  pinMode(enaPin, OUTPUT);     // Motor control pins as outputs
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  digitalWrite(enaPin, LOW);    // Initialize motor pins
  attachInterrupt(digitalPinToInterrupt(irSensorPin), countPulses, RISING);  // Increase counter when speed sensor goes high
  lcd.init();                   // Initialize LCD
  lcd.backlight();
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
}

void loop() {
  double now = millis();
  dt = (now - last_time) / 1000.0;
  last_time = now; 

  float RPM = (countPulse / 2) * 60; // Calculate RPM based on a rotation
  countPulse = 0;

  int Actual_RPM = RPM;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Actual RPM: ");
  lcd.print(Actual_RPM);
  lcd.setCursor(0, 1);
  lcd.print("Setpnt: ");
  lcd.print(setpoint);

  Serial.print("Setpoint: ");
  Serial.println(setpoint);
  Serial.print("PID response: ");
  Serial.println(filtered_signal_Out);
 
  char customKey = customKeypad.getKey();

  if (customKey) {
    switch (customKey) {
      case '0':
        setpoint = 0;    // Minimum speed
        break;
      case '1':
        setpoint = 128;  // Speed level 2
        break;
      case '2':
        setpoint = 250;  // Maximum speed
        break;
      default:
        break;
    }
  }
  
  error = setpoint - Actual_RPM;
  integral += error * dt;
  double derivative = (error - previous_error) / dt;
  signal_Out = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;

  // Apply IIR low-pass filter
  filtered_signal_Out = alpha * signal_Out + (1 - alpha) * filtered_signal_Out;

  // Limit the output to PWM range 0-255
  if (filtered_signal_Out > 255) filtered_signal_Out = 255;
  if (filtered_signal_Out < 0) filtered_signal_Out = 0;

  analogWrite(enaPin, setpoint); // Set motor speed based on PID output
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
}

void countPulses() {
  countPulse++;
}
