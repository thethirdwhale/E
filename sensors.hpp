#ifndef _SENSORS_
#define _SENSORS_

//sensors task
void sensorUpdate(void* ignore);
void changeHeading(double bearing);
void motorTare();

extern double encdL, encdR, speedL, speedR, avgSpeed, currentBearing, currentAngle, bearingChange;

#endif