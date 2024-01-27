#include "autoncontrol.hpp"
#include "main.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "sensors.hpp"
#include "odom.hpp"
#include "variables.hpp"

//externs
double controlSpeedCap = defaultSpeedCap, controlKP = defaultKP, controlKD = defaultKD, controlKI = defaultKI, controlTurnKP = defaultTurnKP, controlTurnKD = defaultTurnKD, controlTurnKI = defaultTurnKI, controlRampingMax = defaultRampingMax;
//wheels
double controlPowLeft, controlPowRight, controlTargPowLeft = 0, controlTargPowRight = 0, controlTargLeft = 0, controlTargRight = 0, controlErrorLeft, controlErrorRight, controlPrevErrorLeft = 0, controlPrevErrorRight = 0, controlDerivLeft, controlDerivRight;
//orientation
double controlTargBearing = 0/*, controlErrorBearing, controlPrevErrorBearing = 0, controlDerivBearing*/;
//other
bool controlPIDEnable = false, controlTurnMode = false, controlRunning = false/*, controlTurnLeft*/;

void controlPID(void *ignore){
    Motor leftFM(leftFMPort,E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor leftMM(leftFMPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor leftRM(leftRMPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor rightFM(rightFMPort, E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);
	Motor rightMM(rightMMPort, E_MOTOR_GEAR_GREEN, true,E_MOTOR_ENCODER_DEGREES);
	Motor rightRM(rightRMPort, E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);
    Imu inertial(IMUPORT);
    Controller master (CONTROLLER_MASTER);

    double deltaLeft, deltaRight;

    leftFM.tare_position();
    leftMM.tare_position();
    leftRM.tare_position();
    rightFM.tare_position();
    rightMM.tare_position();
    rightRM.tare_position();

    while(true){
        if (controlPIDEnable && !inertial.is_calibrating()){
            controlErrorLeft = controlTargLeft - encdL;
            controlErrorRight = controlTargRight - encdR;
            
            controlDerivLeft = controlErrorLeft - controlPrevErrorLeft;
            controlDerivRight = controlErrorRight - controlPrevErrorRight;
            
            controlPrevErrorLeft = controlErrorLeft;
            controlPrevErrorRight = controlErrorRight;

            if(controlTurnMode){
                controlTargPowLeft = controlErrorLeft*controlTurnKP + controlDerivLeft*controlTurnKD;
                controlTargPowRight = controlErrorRight*controlTurnKP + controlDerivRight*controlTurnKD;

                if(fabs(controlErrorLeft) < defaultBearingTolerance && fabs(controlErrorRight) < defaultBearingTolerance){
                    controlTargPowLeft  = absadd(controlTargLeft, controlTurnKI);
                    controlTargPowRight = absadd(controlTargRight, controlTurnKI); 
                }

                // controlErrorBearing = controlTargBearing - sensorsBearing;
            }else{
                controlTargPowLeft = controlErrorLeft*controlKP + controlDerivLeft*controlKD;
                controlTargPowRight = controlErrorRight*controlKP + controlDerivRight*controlKD;
            
                if(fabs(controlErrorLeft) < defaultDistanceTolerance && fabs(controlErrorRight) < defaultDistanceTolerance){
                    controlTargPowLeft = absadd(controlTargPowLeft, controlKI);
                    controlTargPowRight = absadd(controlTargPowRight, controlKI);
                }
            }

            deltaLeft = controlTargPowLeft - controlPowLeft;
            deltaRight = controlTargPowRight - controlPowRight;

            controlPowLeft += cap(deltaLeft, controlRampingMax);
            controlPowRight += cap(deltaRight, controlRampingMax);
            controlPowLeft = cap(controlPowLeft, controlSpeedCap);
            controlPowRight = cap(controlPowRight, controlSpeedCap);
            controlDrive(controlPowLeft, controlPowRight);
            controlRunning = true;
        }

        delay(5);
    }
}


//basic movement
void controlDrive(double left, double right){
    Motor leftFM(leftFMPort, false);
	Motor leftMM(leftFMPort, false);
	Motor leftRM(leftRMPort, false);
	Motor rightFM(rightFMPort, true);
	Motor rightMM(rightMMPort, true);
	Motor rightRM(rightRMPort, true);
    
    leftFM.move(left);
    leftMM.move(left);
    leftRM.move(left);
    rightFM.move(right);
    rightMM.move(right);
    rightRM.move(right);
}


//relative movements
bool controlMove(double inches, double timeout, double kp, double kd, double ki){
    double start = millis();
    controlTurnMode = false;
    printf("controlMove | inches: %f\ttimeout: %f\n", inches, timeout);

    controlKP = kp;
    controlKD = kd;
    controlKI = ki;

    controlTargLeft += inches*degperinch;
    controlTargRight += inches*degperinch;

    delay(50);
    while(fabs(controlErrorLeft) > defaultDistanceTolerance || fabs(controlErrorRight) > defaultDistanceTolerance || fabs(avgSpeed) > defaultVelocityTolerance){
        if (millis() - start > timeout){
            printf("Timeout\n");
            return false;
        }
        printf("Moving\n");
        delay(50);
    }

    return true;
}

bool controlTurn(double degrees, double timeout, double kp, double kd, double ki){
    double start = millis();
    controlTurnMode = true;
    printf("controlTurn | degrees: %f\ttimeout: %f\n", degrees, timeout);

    controlTurnKP = kp;
    controlTurnKD = kd;
    controlTurnKI = ki;
    controlTargLeft += degrees*degperdegrot;
    controlTargRight -= degrees*degperdegrot;
    controlTargBearing = boundDeg(controlTargBearing + degrees);

    delay(50);
    while(fabs(controlErrorLeft) > defaultBearingTolerance || fabs(controlErrorRight) > defaultBearingTolerance || fabs(speedL) > defaultVelocityTolerance || fabs(speedR) > defaultVelocityTolerance){
        if (millis() - start > timeout){
            printf("Timeout\n");
            return false;
        }
        printf("Turning\n");
        delay(50);
    }
    return true;
}


//absolute movements
bool controlMoveTo(bool backwards, double x, double y, double turnTimeout, double moveTimeout, double moveKP, double moveKD, double moveKI, double turnKP, double turnKD, double turnKI){
    printf("controlMoveTo | x: %f\ty: %f\ttimeout: %f, %f\n", x, y, turnTimeout, moveTimeout);
    Controller master(CONTROLLER_MASTER);

    bool success = false;
    double diffX = x - odomGlobalX, diffY = y - odomGlobalY;
    double distance = sqrt(diffX*diffX + diffY*diffY);
    double angle = atan2(diffY, diffX);
    double bearing = boundDeg(90 - angle*toDegree);

    master.print(0, 0, "%f %f", diffX, diffY);
    master.print(1, 0, "%f %f", bearing, distance);

    if(backwards){
        success = controlTurnTo(boundDeg(bearing + 180), turnTimeout, turnKP, turnKD, turnKI);
        success &= controlMove(-distance, moveTimeout, moveKP, moveKD, moveKI);
    } else {
        success = controlTurnTo(bearing, turnTimeout, turnKP, turnKD, turnKI);
        success &= controlMove(distance, moveTimeout, moveKP, moveKD, moveKI);
    }
    return success;
}

bool controlTurnTo(double bearing, double timeout, double kp, double kd, double ki){
    double start = millis();
    controlTurnMode = true;
    printf("controlTurnTo | bearing: %f\ttimeout: %f\n", bearing, timeout);

    double errorBearing = boundDeg(bearing - currentBearing);
    if(errorBearing > 180) errorBearing -= 360;

    controlTurnKP = kp;
    controlTurnKD = kd;
    controlTurnKI = ki;
    controlTargLeft += errorBearing*degperdegrot;
    controlTargRight -= errorBearing*degperdegrot;
    controlTargBearing = bearing;

    delay(50);
    while(fabs(controlErrorLeft) > defaultBearingTolerance || fabs(controlErrorRight) > defaultBearingTolerance || fabs(speedL) > defaultVelocityTolerance || fabs(speedR) > defaultVelocityTolerance){
        if (millis() - start > timeout){
            printf("Timeout\n");
            return false;
        }
        printf("Turning\n");
        delay(50);
    }
    return true;
}

bool controlTurnLeftTo(double bearing, double timeout, double kp, double kd, double ki){
    double start = millis();
    controlTurnMode = true;
    printf("controlTurnLeftTo | bearing: %f\ttimeout: %f\n", bearing, timeout);

    double errorBearing = bearing - currentBearing;
    if (errorBearing > 0) errorBearing -= 360;

    controlTurnKP = kp;
    controlTurnKD = kd;
    controlTurnKI = ki;
    controlTargLeft -= errorBearing*degperdegrot;
    controlTargRight += errorBearing*degperdegrot;
    controlTargBearing = bearing;

    delay(50);
    while(fabs(controlErrorLeft) > defaultBearingTolerance || fabs(controlErrorRight) > defaultBearingTolerance || fabs(speedL) > defaultVelocityTolerance || fabs(speedR) > defaultVelocityTolerance){
        if (millis() - start > timeout){
            printf("Timeout\n");
            return false;
        }
        printf("Turning\n");
        delay(50);
    }
    return true;
}

bool controlTurnRightTo(double bearing, double timeout, double kp, double kd, double ki){
    double start = millis();
    controlTurnMode = true;
    printf("controlTurnRightTo | bearing: %f\ttimeout: %f\n", bearing, timeout);

    double errorBearing = bearing - currentBearing;
    if (errorBearing < 0) errorBearing += 360;

    controlTurnKP = kp;
    controlTurnKD = kd;
    controlTurnKI = ki;
    controlTargLeft -= errorBearing*degperdegrot;
    controlTargRight += errorBearing*degperdegrot;
    controlTargBearing = bearing;

    delay(50);
    while(fabs(controlErrorLeft) > defaultBearingTolerance || fabs(controlErrorRight) > defaultBearingTolerance || fabs(speedL) > defaultVelocityTolerance || fabs(speedR) > defaultVelocityTolerance){
        if (millis() - start > timeout){
            printf("Timeout\n");
            return false;
        }
        printf("Turning\n");
        delay(50);
    }
    return true;
}

void controlSetCoords(double x, double y, double bearing){
    printf("controlSetCoords | x: %f\ty: %f\tbearing: %f\n", x, y, bearing);

    Motor leftFM(leftFMPort,E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor leftMM(leftFMPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor leftRM(leftRMPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor rightFM(rightFMPort, E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);
	Motor rightMM(rightMMPort, E_MOTOR_GEAR_GREEN, true,E_MOTOR_ENCODER_DEGREES);
	Motor rightRM(rightRMPort, E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);

    motorTare();
    odomSetCoords(x, y, bearing);
    controlTargLeft = 0;
    controlTargRight = 0;
    controlTargBearing = boundDeg(bearing);
    odomPrevPosLeft = 0;
    odomPrevPosRight = 0;
    delay(50);
}