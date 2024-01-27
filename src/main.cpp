#include "main.h"
#include "pros/motors.h"
#include "mechlib.cpp"
#include "autonpath.cpp"
//#include "auton.cpp"
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	Motor leftFM(leftFMPort,E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor leftMM(leftFMPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor leftRM(leftRMPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor rightFM(rightFMPort, E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);
	Motor rightMM(rightMMPort, E_MOTOR_GEAR_GREEN, true,E_MOTOR_ENCODER_DEGREES);
	Motor rightRM(rightRMPort, E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);
	//Motor rotation(8,E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);
	//Motor intake(intakePort,  E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_DEGREES);
	//Motor intakeRight(intakeRightPort,  E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_DEGREES);
	Motor catapult(catapultPort,  E_MOTOR_GEAR_RED, false, E_MOTOR_ENCODER_DEGREES);
	Motor intake(intakePort,  E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	//Motor intakeb(8,  E_MOTOR_GEAR_RED, true, E_MOTOR_ENCODER_DEGREES);
	Controller master (E_CONTROLLER_MASTER);
	ADIDigitalOut wing1 ('A');
	ADIDigitalOut wing2 ('B');
	Rotation rotation (rotationPort);
	//Task catapultPIDTask (catapultPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "catapultPIDTask");
	// Task autonPIDTask (autonPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "autonPIDTask");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	//full();
	//Motor catapult(catapultPort, false);
	//Motor claw (clawPort, false);
	
	balls();
	// leftFM.move(-127);
	// leftMM.move(-127);
	// leftRM.move(-127);
	// rightMM.move(-127);
	// rightFM.move(-127);
	// rightRM.move(-127);
	// delay(5000);
	// leftFM.move(0);
	// leftMM.move(0);
	// leftRM.move(0);
	// rightMM.move(0);
	// rightFM.move(0);
	// rightRM.move(0);
	// //claw.move(-127);
	
	// full();
	// calibration();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	Motor leftFM(leftFMPort);
	Motor leftMM(leftMMPort);
	Motor leftRM(leftRMPort);
	Motor rightFM(rightFMPort);
	Motor rightMM(rightMMPort);
	Motor rightRM(rightRMPort);
	Motor intake(intakePort);
	//Motor intakeb(4, true);
	Motor catapult(catapultPort);
	Controller master (E_CONTROLLER_MASTER);
	Rotation rotation (rotationPort);
	//Motor claw (clawPort, false);
	ADIDigitalOut wing1 ('A');
	ADIDigitalOut wing2 ('B');

	double left = 0;
	double right = 0;
	bool inverted = false, wingToggle = false;
	bool pneumaticsactivated = false;
	double error;
	double prevError = 200;
	double deriv;
	double cataMove;
	double kp = 7;
	double kd = 3.1;
	//bool precision = true;
	double targ = 209;//use the rotation sensor value
	wing1.set_value(LOW);
	wing2.set_value(LOW);

	while (true) 
	{
		//if (master.get_digital_new_press(DIGITAL_R2)){
			//inverted = !inverted;
			//master.clear();
		//}

		/*if (master.get_digital_new_press(DIGITAL_R2)){ //Feed
			intakeToggle = !intakeToggle;
		}
		if (master.get_digital_new_press(DIGITAL_Y)) {
			precision = !precision;
		}	*/

		left = master.get_analog(ANALOG_LEFT_Y);
		right = master.get_analog(ANALOG_RIGHT_Y);
		/*if (left>100)
		{
			left = 100;
		}
		if (right>100)
		{
			right = 100;
		}*/
		// if (inverted)
		// {
		// 	//if (precision){
		// 	/*leftFM.move(-right/2);
		// 	leftBack.move(-right/2);
		// 	rightFM.move(-left/2);
		// 	rightBack.move(-left/2);
		// 	master.print(0, 0, "Precision Inverted");
		// 	*/
		// //} else {
		// 	leftFM.move(right);
		// 	leftMM.move(right);
		// 	leftRM.move(right);
		// 	rightFM.move(-left);
		// 	rightMM.move(-left);
		// 	rightRM.move(-left);
		// 	//master.print(0, 0, "Inverted");
		
		// }
		//  else 
		if (left <= 3 && left >= -3){
			leftFM.brake();
			leftMM.brake();
			leftRM.brake();
		} else {
			leftFM.move(left);
			leftMM.move(left);
			leftRM.move(left);
		}
		if (right <= 3 && right >= -3){	
			rightFM.brake();
			rightMM.brake();
			rightRM.brake();
		}		
		else {
			//if (precision){
			//leftFM.move(left/2);
			//leftBack.move(left/2);
			//rightFM.move(right/2);
			//rightBack.move(right/2);
			//master.print(0, 0, "Precision ");
			
		//} else {	
			
			rightFM.move(right);
			rightMM.move(right);
			rightRM.move(right);
			//master.print(0, 0, "Normal");
		} 
		
				
		
		if (master.get_digital_new_press(DIGITAL_R1))
		{
			catapult.move(127);
			delay(250);
			}
		if (master.get_digital_new_press(DIGITAL_L1)){
			wingToggle = ! wingToggle;

		}
		wing1.set_value(wingToggle);
		wing2.set_value(wingToggle);			
		intake.move(127 * (master.get_digital(E_CONTROLLER_DIGITAL_L2) - master.get_digital(E_CONTROLLER_DIGITAL_R2)));
		//if (master.get_digital_new_press(DIGITAL_X)){
			//inverted= !inverted;	
			//rotation.set_value(HIGH);
			//delay(500);
			//rotation.set_value(LOW);
			//delay(500);
			//rotation.move(0);
			
		//}
		/*if (master.get_digital(DIGITAL_R1)){ //Catapult
			catapult.move(127);
			//printf("Shot\n");
		}
		else {
			catapult.move(0);
		}*/

		prevError = error; 
  		error = targ - (rotation.get_position()/100);
		deriv = error - prevError;
 		cataMove = (error * kp + deriv * kd);
		// if (cataMove <= 18){
		// 	cataMove = 18;
		// }
		// if (error >= 2){
		// 	if (error <= -15){
		// 	cataMove = 0;
		// 	}
		// }
		
		catapult.move(cataMove);
		

		

		delay(20);
	}
}

