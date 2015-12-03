//M-Fly Flight Control Software

// NOTE: Servos do work, but interrupt input from
// receiver frequently drops, causing servos to jitter
// uncontrollably

#include <XBee.h>
#include <stdlib.h>

#include <Adafruit_Sensor.h>

#include <Adafruit_BMP085_U.h>
#include <Adafruit_LSM303_U.h>//Accel/Magneto sensor library
#include <Adafruit_L3GD20_U.h>//Gyroscope library
#include <SFE_BMP180.h>

#include "Data.h"
#include <Wire.h>
#include <Servo.h>
//#include "Pressure.h"

Data *data;

volatile uint8_t numDropped = 0;

// Hertz Rate for Data Collection
const int hertz = 2;
const int delayTime = 1000 / hertz;

const long debounceTime = 20000;

// Variables for Data Collection
float batteryVoltage = 9;

//Interrupt Pin
const int interruptPin = 3;
volatile long lastTime = 0; 
volatile long pulseWidth = 0;

Servo doorServo1;
const int doorServo1Pin = 9;

Servo doorServo2;
const int doorServo2Pin = 10;

Servo releaseServo;
const int releaseServoPin = 11;

//PWM min - 870 PWM max - 2100

const int PWM_MAX = 1960;
const int PWM_MIN = 1250;

const int ledPin = 13;
byte ledState = 0;

bool dropped = false;

// XBee Setup

XBee xbee;
XBeeAddress64 broadcast = XBeeAddress64(0x00000000, 0x0000ffff);

void setup() {  
  // Initiate Serial Port
  Serial.begin(9600);
  xbee.begin(Serial);
  
  // Create Data class instance
  data = new Data();
  data->update();

  // Initiate Servos

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

long lastLoopTime = 0;

void loop() {
  while (Serial.available())
    Serial.print((char) Serial.read());
  
  data->update();
  
  // Blink LED and Write Data to Serial regularly
  if (millis() - lastLoopTime > delayTime) {
    lastLoopTime = millis();
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
    
    writeData();
  }
  
  // Disable interrupts to get volatile parameters
  noInterrupts();
  
  bool localDropped = dropped;
  int localWidth = pulseWidth;
  
  interrupts();
  
  // Update Servos after 1000 milliseconds to ensure no false
  // readings from the receiver  
  if (millis() > (long)1000)
    updateDropAndDoorServos(localWidth);
  
  if (dropped) {
    int alt = data->getAltitude();
    
    static char bBuffer[32];
    
    sprintf(bBuffer, "B,%d,%d,\n", numDropped, alt);
    
    ZBTxRequest zbtx = ZBTxRequest(broadcast, (uint8_t *) bBuffer, strlen(bBuffer));
    xbee.send(zbtx);
    
    dropped = false;
  }
}

void writeData() {
  static char csvBuffer[64];
  
  // Write telemetry to serial
  float batteryVoltage = readVcc() / 1000.0f;
  
  float time = millis() / 1000.0f;
  
  char timeBuffer[20];
  char altBuffer[16];
  char gXBuffer[16];
  char gYBuffer[16];
  char gZBuffer[16];
  char battBuffer[16];
  
  sprintf(csvBuffer, "A,M-Fly,%s,%s,%s,%s,%s,%d,%s,%d,\n",
          dtostrf(time, 2, 2, timeBuffer),
          dtostrf(data->getAltitude(), 2, 2, altBuffer),
          dtostrf(data->getGyroX(), 2, 2, gXBuffer),
          dtostrf(data->getGyroY(), 2, 2, gYBuffer),
          dtostrf(data->getGyroZ(), 2, 2, gZBuffer),
          random(10,40), // Airspeed
          dtostrf(batteryVoltage, 2, 2, battBuffer),
          numDropped);
  
  //Serial.print(csvBuffer);
  
  ZBTxRequest zbtx = ZBTxRequest(broadcast, (uint8_t *) csvBuffer, strlen(csvBuffer));
  xbee.send(zbtx);
  
  /*
  Serial.print("A,");
  Serial.print("M-Fly");
  Serial.print(",");
  Serial.print(sprintf("%0.1f",millis()/1000.0));
  Serial.print(",");
  Serial.print(sprintf("%0.2f",data->getAltitude()));
  Serial.print(",");
  Serial.print(data->getGyroX());
  Serial.print(",");
  Serial.print(data->getGyroY());
  Serial.print(",");
  Serial.print(data->getGyroZ());
  Serial.print(",");
  Serial.print(random(10,314));   //Serial.println(MPXV7002DP.GetAirSpeed());
  Serial.print(",");
  Serial.print(batteryVoltage);
  Serial.print(",");
  Serial.print(numDropped);
  Serial.println();
  */
}

void updateDropAndDoorServos(int localPulseWidth) {
  const int MAX = 255;

  // Map value from receiver to usable values
  int val = map(localPulseWidth, PWM_MIN, PWM_MAX, 0, MAX);
  
  val = constrain(val, 0, MAX);
  
  // TODO: Possibly change integer division to floating division
  // based on testing parameters
  
  static int doorWrite = 0;
  int newDoorWrite = 0;
   
  // Open door if val > MAX / 4
  if (val > MAX / 4) {
    newDoorWrite = 180;
  }
  
  // Write to door servos
  if (doorWrite != newDoorWrite) {
    doorWrite = newDoorWrite;
    doorServo1.write(doorWrite);
    doorServo2.write(doorWrite);
  }
  
  // hold values to write to servo
  static int releaseWrite = 0;
  int newReleaseWrite = 0;
  
  // Drop payloads as specified
  if (val > MAX * 3 / 4) {
    newReleaseWrite = 180;
    if (numDropped < 2) {
      numDropped = 2;
      dropped = true;
    }
  } else if (val > MAX / 3) {
    newReleaseWrite = 90;
    if (numDropped < 1) {
      numDropped = 1;
      dropped = true;
    }
  } else if (numDropped >= 2 && val > MAX / 3) {
    ++numDropped;
    dropped = true;
  }
  
  // Only write to servo if necessary to avoid jitter
  if (releaseWrite != newReleaseWrite) {
    releaseWrite = newReleaseWrite;
    releaseServo.write(releaseWrite);
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

// Interrupt for rising edge
void rising(){
  if (micros() - lastTime > debounceTime) {
    lastTime = micros();
  
    detachInterrupt(digitalPinToInterrupt(interruptPin));
    attachInterrupt(digitalPinToInterrupt(interruptPin), drop, FALLING);
  }
}

// Interrupt for falling edge
void drop(){ //writes the drop agle to the servo and logs current flight data at that instant
  pulseWidth = micros() - lastTime;
  pulseWidth = constrain(pulseWidth, PWM_MIN, PWM_MAX);
  
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  attachInterrupt(digitalPinToInterrupt(interruptPin), rising, RISING);
}

