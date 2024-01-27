/*#include "auton.hpp"
#include "main.h"
Motor leftFM(leftFMPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
Motor leftMM(leftMMPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
Motor leftRM(leftRMPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
Motor rightFM(rightFMPort, E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);
Motor rightMM(rightMMPort, E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);
Motor rightRM(rightRMPort,E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);
//Motor intake(intakePort,  E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
//Motor feedRight(feedRightPort,  E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_DEGREES);
//Motor catapultLeft(catapultLeftPort,  E_MOTOR_GEAR_RED, false, E_MOTOR_ENCODER_DEGREES);
//Motor catapult(catapultPort,  E_MOTOR_GEAR_RED, true, E_MOTOR_ENCODER_DEGREES);

double targLeft, targRight, posLeft, posRight, prevErrorLeft, prevErrorRight, errorLeft, errorRight, derivLeft, derivRight, leftSpeed, rightSpeed;
bool targReach;
double degperinch = 35.257; //degrees per inch moved
double degperdegrot = 9.8463; //degrees per degree (of hull) rotation

//width between middle of wheels: 32cm
//circumference of wheels: 10.210cm
//one radian: 57.2958
Task autonPID(void* ignore){
  leftFM.tare_position();
  leftMM.tare_position();
  leftRM.tare_position();
  rightFM.tare_position();
  rightMM.tare_position();
  rightRM.tare_position();
  // posLeft = leftFM.get_position();
  // posRight = rightFM.get_position();
  // errorLeft = targLeft - posLeft;
  // errorRight = targRight - posRight;
  // derivLeft = errorLeft - prevErrorLeft;
  // derivRight = errorRight - prevErrorRight;
  // rightSpeed = errorRight * akp + derivRight * akd;
  // leftSpeed = errorLeft * akp + derivLeft * akd;
  // leftFM.move(leftSpeed);
  // leftMM.move(leftSpeed);
  // leftRM.move(leftSpeed);
  // rightFM.move(rightSpeed);
  // rightMM.move(rightSpeed);
  // rightRM.move(rightSpeed);

  

  while (true && competition::is_autonomous()) {
    
    posLeft = leftFM.get_position();
    posRight = rightFM.get_position();
    errorLeft = targLeft - posLeft;
    errorRight = targRight - posRight;
    derivLeft = errorLeft - prevErrorLeft;
    derivRight = errorRight - prevErrorRight;
    rightSpeed = errorRight * akp + derivRight * akd;
    leftSpeed = errorLeft * akp + derivLeft * akd;
    leftFM.move(leftSpeed);
    leftMM.move(leftSpeed);
    leftRM.move(leftSpeed);
    rightFM.move(rightSpeed);
    rightMM.move(rightSpeed);
    rightRM.move(rightSpeed);



    prevErrorLeft = errorLeft;
    prevErrorRight = errorRight;

    // prevErrorLeft = errorLeft;
    // prevErrorRight = errorRight;

    targReach = errorLeft<3 && errorLeft>-3 && errorRight<=3 && errorRight>=-3;

    //printf("A    Left: %f %f    Right: %f %f\n", errorLeft, leftSpeed, errorRight, rightSpeed);
    delay(20);
  }
}


void move(double inches)
{
  targLeft += inches * degperinch;
  targRight += inches * degperinch;
  
  //delay(5000);
  //return;
  if (targReach)
  {
    return;

  }
}

/*void move(double inches){
  targLeft += inches*akm;
  targRight += inches*akm;

  printf("move %f", inches);
  delay(500);
  if (targReach){
    return;
  } else {
    printf("moving");
  }
}*/
/*
void turn(double degrees)
{
  targLeft += degrees*degperdegrot;
  targRight -= degrees*degperdegrot;
  
  
  if (targReach){
    return;
  }
}

void full(){ 
  Task autonPID (autonPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "autonPID");
  //feedLeft.move(100);
  //feedRight.move(100);
  //leftFM.move(50);
  //leftBack.move(50);
  //rightFM.move(50);
  //rightBack.move(50); //Does the roller, base moves to apply pressure on roller
  //delay(500);
  //feedLeft.move(0);
  //feedRight.move(0);
  ///leftFM.move(0);
  //leftBack.move(0);
  //rightFM.move(0);
  //rightBack.move(0); //Stop

  //turn(90);
  //move(36);
  // turn(-90);
  // move(36);
  // turn(-45);
  // catapultTarg();
  // turn(90);
  // move(12);
  // turn(135);
  // catapultTarg();
  // turn(45);
  // move(48);
  // turn(135);
  // catapultTarg();

  //idk what to add
  autonPID.remove();
}

//void calibration(){
  //Task autonPIDTask (autonPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "autonPIDTask");
  //move(12);
  //autonPIDTask.remove();
  //printf("Task removed");
//}
 
*/
#include "autoncontrol.hpp"
#include "main.h"
#include "odom.hpp"
#include "sensors.hpp"


bool calibration(int path){
    Task autonSensorsTask(sensorUpdate, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sensors Task");
    Task autonOdomTask(odomTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
    Task autonPIDTask(controlPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PID Task");
    
    bool success = false;
    controlPIDEnable = true;
    

    switch (path){
        case 0:
            success = controlMove(12, 3000);
            controlSetCoords(12, 0, -90);
            success &= controlMoveTo(false, 0, 0, 2000, 2000);
            break;

        case 1:
            success = controlTurn(180, 3000);
            break;

        case 2:
            success = controlMove(12, 3500);
            success &= controlMoveTo(false, -12, 12, 1000, 1000);
            success &= controlMoveTo(false, 0, 0, 1000, 3000);
            break;
    }

    controlPIDEnable = false;
    
    autonPIDTask.remove();
    autonOdomTask.remove();
    autonSensorsTask.remove();

    return success;
}

void matchload(){
    Controller master(CONTROLLER_MASTER);
    Task autonSensorsTask(sensorUpdate, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sensors Task");
    Task autonOdomTask(odomTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
    Task autonPIDTask(controlPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PID Task");
  
	Motor intake(intakePort, false);
	Motor cata(catapultPort, false);
	ADIDigitalOut wingLeft('A', false);
	ADIDigitalOut wingRight('B', false);
    controlPIDEnable = true;

    

    controlMove(26, 1000);
    controlTurnTo(45, 1000);
    controlSpeedCap = 60;
    intake.move(80);
    controlMove(5, 100);
    intake.move(0);
    controlMove(25, 650);
    controlMove(-30, 1000);
    controlSetCoords(0, 28, 45);
    delay(50);
    master.print(0, 0, "%f %f", odomGlobalX, odomGlobalY);
    controlSpeedCap = 120;
    controlTurnTo(0, 1250);
    controlMove(-8, 500);
    wingLeft.set_value(true);
    controlMoveTo(true, 0, 10, 100, 1000);
    // controlTurnTo(-30, 500);
    wingLeft.set_value(false);
    wingRight.set_value(true);
    // controlMoveTo(true, 15, -17, 500, 1000);
    controlMoveTo(true, 16, -18, 500, 1000);
    controlPIDEnable = false;
    controlDrive(0, -15);

    
    autonPIDTask.remove();
    autonOdomTask.remove();
    autonSensorsTask.remove();
}

void balls(){
    Task autonSensorsTask(sensorUpdate, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sensors Task");
    Task autonOdomTask(odomTracker, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
    Task autonPIDTask(controlPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PID Task");
    
	Motor intake(intakePort, false);
	Motor cata(catapultPort, false);
	ADIDigitalOut wingLeft('A', false);
	ADIDigitalOut wingRight('B', false);
    controlPIDEnable = true;

    

    controlMove(24, 3000);
    controlMoveTo(false, -24, 48, 1000, 1000);
    controlTurnTo(-90, 1000);
    wingRight.set_value(true);
    intake.move(80);
    controlMove(-5, 100);
    intake.move(0);
    controlMove(-25, 900);

    controlPIDEnable = false;
    
    autonPIDTask.remove();
    autonOdomTask.remove();
    autonSensorsTask.remove();
}