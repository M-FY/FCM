//M-Fly Flight Control Software

#include <Adafruit_Sensor.h>

#include <Adafruit_BMP085_U.h>
#include <Adafruit_LSM303_U.h>//Accel/Magneto sensor library
#include <Adafruit_L3GD20_U.h>//Gyroscope library
#include <SFE_BMP180.h>

#include "Data.h"
#include <Wire.h>
#include <Servo.h>
//#include "Pressure.h"

unsigned long timeOn;
Data *data;

// File for OpenLog
const char filename[] = "mfly.csv";
const char lineEndType = '\n';

// Reset Pin for OpenLog
const int resetPin = 4;

// Hertz Rate for Data Collection
const int hertz = 5;
const int delayTime = 1000 / hertz;

// Variables for Data Collection
float batteryVoltage = 9;

//Interrupt Pin
const int interruptPin = 2;
volatile long lastTime = 0; 
volatile long timeAtRise = 0;

//Servo Definitions
Servo dropServo;
const int servoPin = 9;
const int dropAngles[] = {0, 90};
//PWM min - 870 PWM max - 2100

void setup() {
  // Initiate Serial Port
  Serial.begin(9600);
  
  data = new Data();
  data->update();
  
  // Setup Reset Pin and Rest OpenLog
  pinMode(resetPin, OUTPUT);
  
  digitalWrite(resetPin, LOW);
  delay(100);
  digitalWrite(resetPin, HIGH);

 // Setup OpenLog
  setupOpenLog();

 //Setup servos
 pinMode(servoPin, OUTPUT);
 dropServo.attach(servoPin);

  //attach interupts as the last task in setup
  pinMode(interruptPin, INPUT); //set interruptPin as an input
  digitalWrite(interruptPin,LOW); //set pin to Low initially
  attachInterrupt(digitalPinToInterrupt(interruptPin), rising, RISING); //execute interrupt when voltage drops
}

void setupOpenLog() {
    // Get into OpenLog Command Prompt Mode and create file
  waitForCharacter('<');
  
  // Enter Command Mode
  Serial.write(26);
  Serial.write(26);
  Serial.write(26);
  
  // Create/Append to file
  waitForCharacter('>');
  Serial.print("append ");
  Serial.print(filename);
  Serial.print('\r');
  
  // Print CSV Header
  Serial.print("TEAMID");
  Serial.print(",");
  Serial.print("TIME");
  Serial.print(",");
  Serial.print("Altitude");
  Serial.print(",");
  Serial.print("AccelX");
  Serial.print(",");
  Serial.print("AccelY");
  Serial.print(",");
  Serial.print("AccelZ");
  Serial.print(",");
  Serial.print("GyroX");
  Serial.print(",");
  Serial.print("GyroY");
  Serial.print(",");
  Serial.print("GyroZ");
  Serial.print(",");
  Serial.print("MagX");
  Serial.print(",");
  Serial.print("MagY");
  Serial.print(",");
  Serial.print("MagZ");
  Serial.print(",");
  Serial.print("Airspeed");
  Serial.print(",");
  Serial.print("Voltage");
  Serial.print(lineEndType);
}

void loop() {  
  writeDataToOpenLog();
  
  delay(delayTime);
}

void writeDataToOpenLog() {
  float batteryVoltage = readVcc() / 1000.0f;
  
  Serial.print("M-Fly");
  Serial.print(",");
  Serial.print(timeOn/1000000.0);
  Serial.print(",");
  Serial.print(data->getAltitude());
  Serial.print(",");
  Serial.print(data->getAccelX());
  Serial.print(",");
  Serial.print(data->getAccelY());
  Serial.print(",");
  Serial.print(data->getAccelZ());
  Serial.print(",");
  Serial.print(data->getGyroX());
  Serial.print(",");
  Serial.print(data->getGyroY());
  Serial.print(",");
  Serial.print(data->getGyroZ());
  Serial.print(",");
  Serial.print(data->getMagX());
  Serial.print(",");
  Serial.print(data->getMagY());
  Serial.print(",");
  Serial.print(data->getMagZ());
  Serial.print(",");
  Serial.print("0.0");   //Serial.println(MPXV7002DP.GetAirSpeed());
  Serial.print(",");
  Serial.print(batteryVoltage);
  Serial.print(lineEndType);
}

// Waits until the serial port reads a given character
void waitForCharacter(char c) {
  while (1) {
    if (Serial.available())
      if (Serial.read() == c) return;
  }
}

// Gets Vcc Voltage of Battery
// From: https://code.google.com/p/tinkerit/wiki/SecretVoltmeter
long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

void rising(){
  
  if(micros() - lastTime > 250){ 
    timeAtRise = micros();
  }

  lastTime = micros();

  detachInterrupt(digitalPinToInterrupt(interruptPin));
  attachInterrupt(digitalPinToInterrupt(interruptPin), drop, FALLING);
}

void drop(){ //writes the drop agle to the servo and logs current flight data at that instant

  if(micros() - lastTime > 250){ //ensure against multiple executions of the drop function for one voltage change
    int angle = map(micros()-lastTime,870,2100,0,180);
    dropServo.write(angle);
  }

  lastTime = micros();

  detachInterrupt(digitalPinToInterrupt(interruptPin));
  attachInterrupt(digitalPinToInterrupt(interruptPin), rising, RISING);
}

void retract(){//for possible use later, writes to the servo the retracted angle
  dropServo.write(dropAngles[0]);
}

