/*
Code written 10/16/15
Written by Daniel and James
Function Comments written by Daniel
*/

#include "Data.h"


Data::Data() : accel(30301), mag(30302), gyro(20) {
	accel.begin();
  mag.begin();
  gyro.begin();
	
	pressure.begin();

	setBaseline();
	update();
}

/*
Sets the acceleration (X) variable
Private because it should only be called by update()
inputAccelX - the value to set to the Acceleration
*/
void Data::setAccelX(double inputAccelX) {
	accelX = inputAccelX;
}

/*
Sets the acceleration (Y) variable
Private because it should only be called by update()
inputAccelY - the value to set to the Acceleration
*/
void Data::setAccelY(double inputAccelY) {
	accelY = inputAccelY;
}

/*
Sets the acceleration (Z) variable
Private because it should only be called by update()
inputAccelZ - the value to set to the Acceleration
*/
void Data::setAccelZ(double inputAccelZ) {
	accelZ = inputAccelZ;
}

/*
Returns the (X) Value of the Accerlation
Should be called immediately after update();
Returns: AcccelX as a double
*/
double Data::getAccelX() {
	return accelX;
}

/*
Returns the (Y) Value of the Accerlation
Should be called immediately after update();
Returns: AcccelY as a double
*/
double Data::getAccelY() {
	return accelY;
}

/*
Returns the (Z) Value of the Accerlation
Should be called immediately after update();
Returns: AcccelZ as a double
*/
double Data::getAccelZ() {
	return accelZ;
}

/*
Sets the gyroscope (X) variable
Private because it should only be called by update()
inputGyroX - the value to set to the Gyroscope
*/
void Data::setGyroX(double inputGyroX) {
	gyroX = inputGyroX;
}

/*
Sets the gyroscope (Y) variable
Private because it should only be called by update()
inputGyroY - the value to set to the Gyroscope
*/
void Data::setGyroY(double inputGyroY) {
	gyroY = inputGyroY;
}

/*
Sets the gyroscope (Z) variable
Private because it should only be called by update()
inputGyroZ - the value to set to the Gyroscope
*/
void Data::setGyroZ(double inputGyroZ) {
	gyroZ = inputGyroZ;
}

/*
Returns the (X) Value of the Gyroscope
Should be called immediately after update();
Returns: gyroX as a double
*/
double Data::getGyroX() {
	return gyroX;
}

/*
Returns the (Y) Value of the Gyroscope
Should be called immediately after update();
Returns: gyroY as a double
*/
double Data::getGyroY() {
	return gyroY;
}

/*
Returns the (Z) Value of the Gyroscope
Should be called immediately after update();
Returns: gyroZ as a double
*/
double Data::getGyroZ() {
	return gyroZ;
}

/*
Sets the magnetometer (X) variable
Private because it should only be called by update()
inputMagX - the value to set to the Gyroscope
*/
void Data::setMagX(double inputMagX) {
	magX = inputMagX;
}

/*
Sets the magnetometer (Y) variable
Private because it should only be called by update()
inputMagY - the value to set to the Gyroscope
*/
void Data::setMagY(double inputMagY) {
	magY = inputMagY;
}

/*
Sets the magnetometer (Z) variable
Private because it should only be called by update()
inputMagZ - the value to set to the Gyroscope
*/
void Data::setMagZ(double inputMagZ) {
	magZ = inputMagZ;
}

/*
Returns the (X) Value of the magnetometer
Should be called immediately after update();
Returns: magX as a double
*/
double Data::getMagX() {
	return magX;
}

/*
Returns the (Y) Value of the magnetometer
Should be called immediately after update();
Returns: magY as a double
*/
double Data::getMagY() {
	return magY;
}

/*
Returns the (Z) Value of the magnetometer
Should be called immediately after update();
Returns: magZ as a double
*/
double Data::getMagZ() {
	return magZ;
}


/*
gets the pressure directly from the sensor
method coppied from BMP180 example code
returns - the current pressure as a double
*/
double Data::getPressure()
{
	char status;
	double T, P, p0, a;

	// You must first get a temperature measurement to perform a pressure reading.

	// Start a temperature measurement:
	// If request is successful, the number of ms to wait is returned.
	// If request is unsuccessful, 0 is returned.

	status = pressure.startTemperature();
	if (status != 0)
	{
		// Wait for the measurement to complete:

		delay(status);

		// Retrieve the completed temperature measurement:
		// Note that the measurement is stored in the variable T.
		// Use '&T' to provide the address of T to the function.
		// Function returns 1 if successful, 0 if failure.

		status = pressure.getTemperature(T);
		if (status != 0)
		{
			// Start a pressure measurement:
			// The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
			// If request is successful, the number of ms to wait is returned.
			// If request is unsuccessful, 0 is returned.

			status = pressure.startPressure(3);
			if (status != 0)
			{
				// Wait for the measurement to complete:
				delay(status);

				// Retrieve the completed pressure measurement:
				// Note that the measurement is stored in the variable P.
				// Use '&P' to provide the address of P.
				// Note also that the function requires the previous temperature measurement (T).
				// (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
				// Function returns 1 if successful, 0 if failure.

				status = pressure.getPressure(P, T);
				if (status != 0)
				{
					return(P);
				}
				else Serial.println("error retrieving pressure measurement\n");
			}
			else Serial.println("error starting pressure measurement\n");
		}
		else Serial.println("error retrieving temperature measurement\n");
	}
	else Serial.println("error starting temperature measurement\n");
}

/*
Returns the current Altitude
Should be called immediately after update();
Returns: aaltitude as a double
*/
double Data::getAltitude() {
	return altitude;
}
/*
Sets the altitude variable
Private because it should only be called by update()
inputAlt - the value to set to the Altitude
*/
void Data::setAltitude(double inputAlt) {
	altitude = inputAlt;
}

/*
Sets the baseline
Private because it shouldonly called once
as part of the constructor
*/
void Data::setBaseline() {
	baseline = getPressure();
}

/*
Sets the pressure variable
Private because it should only be called by update()
inputPressure - the value to set to the Pressure
*/
void Data::setPressure(double inputPressure) {
	pressure1 = inputPressure;
}

/*
updates all of the sensor data to the most recent values
calls the sensor's functions to access the values
this should always be called prior to accessing any of the values
*/
void Data::update() {
	sensors_event_t event;
	accel.getEvent(&event);

	setAccelX(event.acceleration.x);
	setAccelY(event.acceleration.y);
	setAccelZ(event.acceleration.z);

	gyro.getEvent(&event);
	setGyroX(event.gyro.x);
	setGyroY(event.gyro.y);
	setGyroZ(event.gyro.z);


	mag.getEvent(&event);
	setMagX(event.magnetic.x);
	setMagY(event.magnetic.y);
	setMagZ(event.magnetic.z);


	setAltitude(pressure.altitude(getPressure(), baseline));
}
