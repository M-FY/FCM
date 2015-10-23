#ifndef __MFLY_DATA
#define __MFLY_DATA

#include <arduino.h>
#include <SFE_BMP180.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>


class Data {
public:

	Data();

	double getAltitude();

	double getAccelX();
	double getAccelY();
	double getAccelZ();

	double getGyroX();
	double getGyroY();
	double getGyroZ();

	double getMagX();
	double getMagY();
	double getMagZ();
	void setBaseline();
	void update();

private:
	double getPressure();
	void setAltitude(double inputAlt);
	void setAccelX(double inputAccelX);
	void setAccelY(double inputAccelY);
	void setAccelZ(double inputAccelZ);
	void setGyroX(double inputGyroX);
	void setGyroY(double inputGyroY);
	void setGyroZ(double inputGyroZ);
	void setMagX(double inputMagX);
	void setMagY(double inputMagY);
	void setMagZ(double inputMagZ);

	void setPressure(double inputPressure);
	sensor_t mySensor;
	double accelX, accelY, accelZ;
	double gyroX,  gyroY,  gyroZ;
	double magX,   magY,   magZ;

	SFE_BMP180 pressure;

	Adafruit_LSM303_Accel_Unified accel;
	Adafruit_L3GD20_Unified gyro;
	Adafruit_LSM303_Mag_Unified mag;

	double baseline,pressure1, altitude;
};

#endif
