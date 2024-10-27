#include "main.h"

Controller master(E_CONTROLLER_MASTER);
Motor left_mg(11);    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
Motor right_mg(-20);  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
Imu imu (5); //initialize imu

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Block until calibration is complete
  imu.reset(true);
  delay(2000);
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



double encoderConvert(double distance) {
	return distance / (3.14 * 4.0) * 900.0;
}




void driveForwards(int time) {
	left_mg.move(127);
	right_mg.move(127);
	delay(time);
	left_mg.move(0);
	right_mg.move(0);
}

void moveEncoder(int distance, double kP, double kD, int time) {
	double startTime = millis();
	double timeElapsed = millis() - startTime;
	left_mg.tare_position();
	double derivative;
	// int error2 = distance - right_mg.get_position();
	int error = distance - left_mg.get_position();
	double past_error = 0;
	while(abs(error) > 5 && timeElapsed < time) {
		timeElapsed = millis() - startTime;
		error = distance - left_mg.get_position();
		derivative = past_error - error;
		left_mg.move(error*kP + derivative*kD);
		right_mg.move(error*kP + derivative*kD);
		past_error = error;
	}
	right_mg.move(0);
	left_mg.move(0);
}
void turnImu(double degrees, double kP, double kD, double time) {
	double startTime = millis();
	double timeElapsed = millis() - startTime;
	imu.tare_rotation();
	double derivative;
	double past_error = 0;
	int error = - degrees + imu.get_rotation();
	while(abs(error) > 1 && timeElapsed < time) { 
		timeElapsed = millis() - startTime;
		error = degrees - imu.get_rotation();
		derivative = past_error - error;
		left_mg.move(-(error*kP + derivative * kD));
		right_mg.move(error*kP + derivative * kD);
	}
	right_mg.move(0);
	left_mg.move(0);

}

int speedCap(double speed) {
	if (speed > 100) {
		return 100;
	} else if (speed < -100) {
		return -100;
	} else {
		return speed;
	}
	
}	

void moveEncoderTime(int distance, double kP, double kD, int time) {
	double startTime = millis();
	double timeElapsed = millis() - startTime;
	double smallTime = millis();
	left_mg.tare_position();
	right_mg.tare_position();
	double derivative;
	double derivative2;
	int error2 = distance - right_mg.get_position();
	int error = distance - left_mg.get_position();
	double past_error = 0;
	double past_error2 = 0;
	while(timeElapsed < time) {
		smallTime = millis();
		timeElapsed = millis() - startTime;
		error = distance - left_mg.get_position();
		error2 = distance - right_mg.get_position();
		derivative = past_error - error;
		derivative2 = past_error2 - error2;
		left_mg.move(error*kP + derivative*kD);
		right_mg.move(error2*kP + derivative*kD);
		past_error = error;
		if(error > 2 && error2 > 2) {
			smallTime = millis();
		}
		if((millis() - smallTime) > 500) {
			break;
		}
	}
	right_mg.move(0);
	left_mg.move(0);
}




void driveStraight(int distance, double kP, double kD, int time, double turnKP) {
	//calibration
	left_mg.tare_position();
	right_mg.tare_position();
	imu.tare_rotation();
	//time
	double target = encoderConvert(distance);
	double startTime = millis();
	double timeElapsed = millis() - startTime;
	double smallTime = millis();

	double derivative;
	double derivative2;

	//error
	int error2 = target - right_mg.get_position();
	int error = target - left_mg.get_position();
	double turningError = imu.get_rotation();
	double past_error = 0;
	double past_error2 = 0;

	while(timeElapsed < time) {
		timeElapsed = millis() - startTime;

		turningError = imu.get_rotation();
		error = target - left_mg.get_position();
		error2 = target - right_mg.get_position();
		derivative = past_error - error;
		derivative2 = past_error2 - error2;
		left_mg.move(speedCap(error*kP + derivative*kD) + turningError * turnKP);
		right_mg.move(speedCap(error2*kP + derivative2*kD) + -turningError * turnKP);

		

		past_error = error;
		if(abs(error) > 2 && abs(error2) > 2 && abs(turningError > 2)) {
			smallTime = millis();
		}
		if((millis() - smallTime) > 3000) {
			break;
		}
	}
	right_mg.move(0);
	left_mg.move(0);
}





void turnImuTime(double degrees, double kP, double kD, double time) {
	double startTime = millis();
	double timeElapsed = millis() - startTime;
	double smallTime = millis();
	imu.tare_rotation();
	double derivative;
	double past_error = 0;
	int error = - degrees + imu.get_rotation();
	while(timeElapsed < time) { 
		timeElapsed = millis() - startTime;
		error = degrees - imu.get_rotation();
		derivative = past_error - error;
		left_mg.move(-(error*kP + derivative * kD));
		right_mg.move(error*kP + derivative * kD);

		if(abs(error) > 2) {
			smallTime = millis();
		}
		if ((millis() - smallTime) > 500) {
			break;
		}

	}
	right_mg.move(0);
	left_mg.move(0);

}



void turn(int distance) {
	left_mg.tare_position();
	// int error2 = distance - right_mg.get_position();
	int error = distance - left_mg.get_position();
	while(abs(error) > 1) {
		error = distance - left_mg.get_position();
		left_mg.move(error);
		right_mg.move(-error);
	}
	right_mg.move(0);
	left_mg.move(0);
}



void turnRight(int amount) {
	for (int i = 0; i < amount; i++){
		delay(1000);
		turnImu(90, 1.2, 1.5, 1500);
		delay(1000);
	}
}


void autonomous() {
	driveStraight(100, 2, 0.5, 10000, 3);
	turnImuTime(90, 3, 1, 10000);
	driveStraight(100, 2, 0.5, 10000, 3);
	turnImuTime(90, 3, 1, 10000);
	driveStraight(100, 2, 0.5, 10000, 3);
	turnImuTime(90, 3, 1, 10000);
	driveStraight(100, 2, 0.5, 10000, 3);
	turnImuTime(90, 3, 1, 10000);
	// moveEncoderTime(1000, 2, 1.5, 2000);
	//1st turn
	// turnRight(-45);
	// delay(1000);
	// moveEncoder(800, 1, 1.5);
	// //2nd turn to align
	// turnRight(90);
	// delay(1000);
	// moveEncoder(1500, 1, 1.5);
	// //3rd 
	// turnRight(-90);
	// delay(1000);
	// moveEncoder(800, 1, 1.5);
	// //4th
	// turnRight(-90);
	// delay(1000);
	// moveEncoder(800, 1, 1.5);
	// //5th
	// turnRight(-90);
	// delay(1000);
	// moveEncoder(800, 1, 1.5);


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
void opcontrol() {
	

	while (true) {
		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = -master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		delay(20);                               // Run for 20 ms then update
	}
}
