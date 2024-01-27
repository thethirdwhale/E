/*
#include "main.h" 
//constants 
//(wheel diameter * pi) / (360 * gear ratio) 
const double INPERDEG = 0.0149599650170952; 
const double pi = 3.141592;
//variables 
double globalX = 0, globalY = 0, localX = 0, localY = 0, prevPos = 0, prevBearing = 0; 
//set current coordinates 
void setCoords (double x, double y) { globalX = x; globalY = y; } 
//main odom loop 
void odometry (void*ignore) { 
    IMU inertial (IMUPORT);
    Controller master (E_CONTROLLER_MASTER);
    resetPrevEncd();
    while (true) { 
        //calibrate inertial 
        if (inertial.is_calibrating()){ 
            resetCoords(0,0); }
        else { 
        //get encd values 
        double encdChangeL = (encdL - prevEncdL) * INPERDEG; 
        double encdChangeR = (encdR - prevEncdR) * INPERDEG; 
        double bearingChange = (bearing - prevBearing) * TORAD; 
        //fix heading 
        if (bearingChange > (TWOPI + HALFPI)) { bearingChange -= TWOPI; } 
        //update prev 
        prevEncdL = encdL; prevEncdR = encdR; prevBearing = bearing; 
        //local change 
        if (bearingChange <= 0.01) { 
            localY = encdChangeL; localX = 0; 
            } 
        else { 
            double radius = (encdChangeR/bearingChange) + DIS; localX = radius * (1 - cos(bearingChange)); localY = radius * sin(bearingChange); 
            } 
            // globalX += localX; 
            // globalY += localY; 
            //global change double avgBearing = (prevBearing * TORAD) + (bearingChange/2);
            globalX += (cos(-avgBearing)*localX) - (sin(-avgBearing)*localY); globalY += (sin(-avgBearing)*localX) + (cos(-avgBearing)*localY); } delay(5); } } 
            //function to reset encd void resetPrevEncd () { prevEncdL = 0; prevEncdR = 0; prevBearing = OFFSET; }

*/

#include "main.h"
#include "odom.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "sensors.hpp"
#include "variables.hpp"

double odomGlobalX = 0, odomGlobalY = 0, globalDeltaY, globalDeltaX, localX, localY, odomRadius, odomPrevPosLeft = 0, odomPrevPosRight = 0, odomDeltaPosLeft, odomDeltaPosRight, odomDeltaInchesLeft, odomDeltaInchesRight, odomPrevAngle, odomDeltaAngle;
bool pause;
void odomTracker(void *ignore){
    printf("Odom tracker started\n");
    IMU imu(IMUPORT);
    Controller master(CONTROLLER_MASTER);

    //Rotating vector
    double ethan;

    //Holding variables
    double odomPosLeft, odomPosRight, odomAngle;

    while(true){
        if(imu.is_calibrating()){
            odomSetCoords(0, 0, 0);
        }else{
            odomPosLeft = encdL;
            odomPosRight = encdR;
            odomAngle = currentAngle;
            odomDeltaPosLeft = odomPosLeft - odomPrevPosLeft;
            odomDeltaPosRight = odomPosRight - odomPrevPosRight;
            odomDeltaInchesLeft = odomDeltaPosLeft/degperinch;
            odomDeltaInchesRight = odomDeltaPosRight/degperinch;
            odomDeltaAngle = odomAngle - odomPrevAngle;
            ethan = -odomPrevAngle;

            if (odomDeltaAngle != 0){
                odomRadius = odomDeltaInchesRight/odomDeltaAngle + rightDist;
                
                // printf("radius: %f\n", radius);
                // printf("x: %f\n", radius - radius*cos(odomDeltaAngle));
                // printf("y: %f\n", sin(odomDeltaAngle)*radius);
                localX = odomRadius - odomRadius*cos(odomDeltaAngle);
                localY = sin(odomDeltaAngle)*odomRadius;

                globalDeltaX = localX*cos(ethan) - localY*sin(ethan);
                globalDeltaY = localX*sin(ethan) + localY*cos(ethan);
            } else {
                localX = 0;
                localY = (odomDeltaPosLeft + odomDeltaPosRight)/2;
                
                globalDeltaX = localX*cos(ethan) - localY*sin(ethan);
                globalDeltaY = localX*sin(ethan) + localY*cos(ethan);
            }

            odomGlobalX += globalDeltaX;
            odomGlobalY += globalDeltaY;

            odomPrevPosLeft = odomPosLeft;
            odomPrevPosRight = odomPosRight;
            odomPrevAngle = odomAngle;
            // master.print(0, 1, "%f", odomAngle);
            master.print(0, 0, "%f %f", odomGlobalX, odomGlobalY);
        }
    }

    delay(5);
}


void odomSetCoords(double x, double y, double bearing){
    odomPrevPosLeft = 0;
    odomPrevPosRight = 0;
    odomPrevAngle = boundRad(bearing*toRadian);
    odomGlobalX = x;
    odomGlobalY = y;
    changeHeading(boundDeg(bearing));
}