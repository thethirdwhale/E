#include "main.h"
#include "sensors.hpp"
#include "variables.hpp"

double encdL, currentAngle, encdChangeL, speedL, encdR, speedR, encdChangeR, avgSpeed, currentBearing, bearingChange;
    Motor leftFM(leftFMPort,E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor leftMM(leftFMPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor leftRM(leftRMPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);   
	Motor rightFM(rightFMPort, E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);
	Motor rightMM(rightMMPort, E_MOTOR_GEAR_GREEN, true,E_MOTOR_ENCODER_DEGREES);
	Motor rightRM(rightRMPort, E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);
    Imu imu(IMUPORT);

    

void sensorUpdate(void *ignore){
    Motor leftFM(leftFMPort,E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor leftMM(leftFMPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor leftRM(leftRMPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor rightFM(rightFMPort, E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);
	Motor rightMM(rightMMPort, E_MOTOR_GEAR_GREEN, true,E_MOTOR_ENCODER_DEGREES);
	Motor rightRM(rightRMPort, E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);
    Imu imu(IMUPORT);
    leftFM.tare_position();
    leftMM.tare_position();
    leftRM.tare_position();
    rightFM.tare_position();
    rightMM.tare_position();
    rightRM.tare_position();
    while (true){
        if(imu.is_calibrating() == false){
            encdL = (leftFM.get_position() + leftMM.get_position() + leftRM.get_position())/3;
            encdR = (rightFM.get_position() + rightMM.get_position() + rightRM.get_position())/3;
            
            speedL = (leftFM.get_actual_velocity() + leftMM.get_actual_velocity() + leftRM.get_actual_velocity())/3;
            speedR = (rightFM.get_actual_velocity() + rightMM.get_actual_velocity() + rightRM.get_actual_velocity())/3;
            avgSpeed = (speedL + speedR)/2;

            currentBearing = boundDeg(imu.get_heading() + bearingChange);
            currentAngle =  boundRad(bearingChange/57.29582);
        }



    }
}
void changeHeading(double bearing){
    bearingChange = currentBearing - bearing;
}
void motorTare(){
    Motor leftFM(leftFMPort,E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor leftMM(leftFMPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor leftRM(leftRMPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor rightFM(rightFMPort, E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);
	Motor rightMM(rightMMPort, E_MOTOR_GEAR_GREEN, true,E_MOTOR_ENCODER_DEGREES);
	Motor rightRM(rightRMPort, E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);

    leftFM.tare_position();
    leftMM.tare_position();
    leftRM.tare_position();
    rightFM.tare_position();
    rightMM.tare_position();
    rightRM.tare_position();
}






