//M-Fly Flight Control Software

#include <XBee.h>

#include <Adafruit_Sensor.h>

#include <Adafruit_BMP085_U.h>
#include <Adafruit_LSM303_U.h>//Accel/Magneto sensor library
#include <Adafruit_L3GD20_U.h>//Gyroscope library
#include <SFE_BMP180.h>

#include "Data.h"
#include <Wire.h>
#include <Servo.h>
//#include "Pressure.h"

XBee xbee = XBee();

char lineEndType = '\n';

Data *data;

// Reset Pin for OpenLog
const int resetPin = 4;

volatile uint8_t numDropped = 0;

// Hertz Rate for Data Collection
const int hertz = 5;
const int delayTime = 1000 / hertz;

// Variables for Data Collection
float batteryVoltage = 9;

//Interrupt Pin
const int interruptPin = 2;
volatile long lastTime = 0; 
volatile long pulseWidth = 0;

Servo doorServo1;
const int doorServo1Pin = 9;

Servo doorServo2;
const int doorServo2Pin = 10;

Servo releaseServo;
const int releaseServoPin = 12;

//PWM min - 870 PWM max - 2100

const int PWM_MAX = 1960;
const int PWM_MIN = 1250;

const int ledPin = 13;
byte ledState = 0;

void setup() {
  // Initiate Serial Port
  Serial.begin(9600);
  
  data = new Data();
  data->update();

  doorServo1.attach(doorServo1Pin);
  doorServo1.write(0);
  
  doorServo2.attach(doorServo2Pin);
  doorServo2.write(0);
  
  releaseServo.attach(releaseServoPin);
  releaseServo.write(0);
  
  pinMode(ledPin, OUTPUT);

  //attach interupts as the last task in setup
  pinMode(interruptPin, INPUT); //set interruptPin as an input
  digitalWrite(interruptPin,LOW); //set pin to Low initially
  attachInterrupt(digitalPinToInterrupt(interruptPin), rising, RISING); //execute interrupt when voltage drops
}


//TOOD -> REMOVE
/*
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
*/

long lastLoopTime = 0;

void loop() {
  
  if (millis() - lastLoopTime > delayTime) {
    lastLoopTime = millis();
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
    
    writeData();
  }
}

void writeData() {
  float batteryVoltage = readVcc() / 1000.0f;
  
  Serial.print("A,");
  Serial.print("M-Fly");
  Serial.print(",");
  Serial.print(millis()/1000);
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
  Serial.print(",");
  Serial.print(numDropped);
  Serial.println();
}

void updateDropAndDoorServos() {
  int localPulseWidth = pulseWidth;
  
  int val = map(localPulseWidth, PWM_MIN, PWM_MAX, 0, 255);
  val = constrain(val, 0, 255);
  
  static int doorWrite = 0;
  int newDoorWrite = 0;
  
  if (val > 255 / 4) {
    newDoorWrite = 180;
  }
  
  if (doorWrite != newDoorWrite) {
    doorWrite = newDoorWrite;
    doorServo1.write(doorWrite);
    doorServo2.write(doorWrite);
  }
  
  static int releaseWrite = 0;
  int newReleaseWrite = 0;
  
  bool dropped = false;
  
  if (val > 255 * 3 / 4) {
    newReleaseWrite = 180;
    if (numDropped < 1) {
      numDropped = 1;
      dropped = true;
    }
  } else if (val > 255 / 2) {
    newReleaseWrite = 90;
    if (numDropped < 2) {
      numDropped = 2;
      dropped = true;
    }
  }
  
  if (dropped) {
    int alt = data->getAltitude();
    Serial.print("B,");
    Serial.print(numDropped);
    Serial.print(',');
    Serial.println(alt); 
  }
  
  if (releaseWrite != newReleaseWrite) {
    releaseWrite = newReleaseWrite;
    releaseServo.write(releaseWrite);
  }
    
  Serial.println(val);
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
  lastTime = micros();

  detachInterrupt(digitalPinToInterrupt(interruptPin));
  attachInterrupt(digitalPinToInterrupt(interruptPin), drop, FALLING);
}

void drop(){ //writes the drop agle to the servo and logs current flight data at that instant
  pulseWidth = micros() - lastTime;
  
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  attachInterrupt(digitalPinToInterrupt(interruptPin), rising, RISING);
  
  lastTime = constrain(pulseWidth, PWM_MIN, PWM_MAX);
}
