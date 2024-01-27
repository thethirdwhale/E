/*#include "main.h"

Motor catapult(catapultPort,  E_MOTOR_GEAR_RED, false, E_MOTOR_ENCODER_DEGREES);
Rotation rotation (rotationPort);
double pos, targ = 113.99, error, prevError, deriv, catapultMovement;
bool shoot = false ;

// double targRight = 0, errorRight, prevErrorRight, derivRight

void catapultPID(void *ignore) {
  // Motor catapultLeft(catapultLeftPort, E_MOTOR_GEAR_RED, false,
  // E_MOTOR_ENCODER_DEGREES);
  Motor catapult(catapultPort, E_MOTOR_GEAR_RED, false, E_MOTOR_ENCODER_DEGREES);
  Rotation rotation(rotationPort, false);

  // catapultRight.tare_position();

  // pos = rotation.get_position()/1000;
  // if (pos>0){
  //   targ+=360;
  //   printf("Rotat %.2f\n", pos);
  // }

  // error = prevError = targ - pos;
  error = prevError; 
  error = targ - catapult.get_position();
  // errorRight = prevErrorRight = targRight - catapultRight.get_position();
  deriv = error - prevError;
  // derivRight = errorRight - prevErrorRight;
  if(error <= 0){
    error = 127;
    deriv = 127;
  }
  catapult.move(error * kp + deriv * kd);
  // catapultRight.move(error"kp + deriv*kd)

  /*while (true) {
    if (master.get_digital(DIGITAL_L1)) {
      catapult.move(115);
      // catapultRight.move(100);
      // targ = rotation.get_position()/100;
      targ = catapult.get_position();
      // targRight = catapultRight.get_position();
      printf("Manual\n");
    } else {
    pos = rotation.get_position()/100.0;
    error = targ - catapult.get_position();
    // errorRight = targRight - catapultRight.get_position(); //Porportion
    deriv = error - prevError;
    // derivRight = errorRight - prevErrorRight; //Deriv

    catapult.move(error * kp + deriv * kd);
      // catapultRight.move(error"kp + deriv*kd) //Move

    prevError = error;
    // prevErrorRight = errorRight; //Prev errors

    printf("%f %f\n", error, error * kp + deriv * kd);
    // printf("Pos: %f", catapult.get_position());
    }
  }*/
//}

//void catapultTarg() {
  //targ += 1080;
  // targRight += 1080;
//}
/*
void catapultPos() {
  prevError = error;
  pos = rotation.get_position();
  error = pos - targ; 
  deriv = error - prevError;
  catapultMovement = error * kp + deriv * kd;
   if (pos = targ) {   
    catapultMovement = 0; 
  }
  if (catapultMovement < 0){
    catapultMovement = 127;
  }
  catapult.move(catapultMovement); 
  }






*/
 // }

//}

//bool isError() {
  //return -5 < error && error < 5 /* 5<errorRight && errorRight<5*/;
//}
