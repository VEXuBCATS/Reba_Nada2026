#include "main.h"
//#include "pros/abstract_motor.hpp"
//#include "pros/imu.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "robotConfigs.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */




//pros::MotorGroup left_mg({-8, 10}, pros::v5::MotorGears::blue);    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
//pros::MotorGroup right_mg({4, -9}, pros::v5::MotorGears::blue);
//pros::MotorGroup elevator({7,-6}, pros::v5::MotorGears::green);
//pros::Motor wrist(11, pros::v5::MotorGears::blue);
//pros::Motor claw(16, pros::v5::MotorGears::red); 
//pros::IMU imu(20);




/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
 /*
void initialize() {
	pros::lcd::initialize();
}
	*/

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
ASSET(rutaPrueba1_txt);
ASSET(rutaPrueba2_txt);
ASSET(path1_txt);
ASSET(path2_txt);

void autonomous() {
	auto2();
}

void auto2() {
	chassis.setPose(0, -64, 0);
	chassis.follow(path1_txt, 18, 4000);
	chassis.turnToHeading(180, 750);

	elevator.move_absolute(5000, 200);
	pros::delay(1500);
	chassis.moveToPoint(0, 0, 1000, {.maxSpeed = 80}, false);

	chassis.moveToPoint(0, 24, 1000, {.forwards = false, .maxSpeed = 80}, true);
	elevator.move_absolute(500, 200);
	pros::delay(1500);

	chassis.turnToHeading(135, 750);
	chassis.follow(path2_txt, 18, 4000);

	chassis.turnToHeading(0, 750);
	chassis.moveToPoint(0, -64, 3000, {.forwards = false, .maxSpeed = 80}, false);
}

void auto1() {
	chassis.setPose(0, -64, 0);
	chassis.follow(rutaPrueba2_txt, 18, 6500);
	chassis.moveToPoint(0, -64, 2000);
	chassis.turnToHeading(0, 1000);
	chassis.moveToPoint(0, -64, 2000, {.forwards = false});
}

void autoTest() {
	chassis.setPose(0, 0, 0);
	claw.set_zero_position(-1000);

	chassis.moveToPoint(0, 32, 1250, {.maxSpeed = 80}, true);
	elevator.move_absolute(2300, 200);
	pros::delay(1000);

	claw.move_absolute(-1000, 100);  
	pros::delay(1000);

	
	chassis.moveToPoint(0, 15, 1250, {.forwards = false, .maxSpeed = 80}, true);
	elevator.move_absolute(200, 200);
	pros::delay(1000);
	//pros::delay(500);
	chassis.turnToHeading(40, 750, {.maxSpeed = 80});
	//pros::delay(500);
	chassis.moveToPoint(11.5, 22.5, 1250, {.maxSpeed = 80}, false);

	chassis.swingToHeading(0, lemlib::DriveSide::LEFT, 750, {.maxSpeed = 80});

	chassis.moveToPoint(11.5, 56.5, 1250, {.maxSpeed = 80}, false);

	chassis.turnToHeading(-90, 750, {.maxSpeed = 80});

	chassis.moveToPoint(-19, 56.5, 1250, {.maxSpeed = 80}, false);

	chassis.turnToHeading(-180, 750, {.maxSpeed = 80});

	chassis.moveToPoint(-19, 25, 1250, {.maxSpeed = 80}, false);

	chassis.moveToPoint(0, 0, 1250, {.maxSpeed = 80}, false);

	chassis.turnToHeading(0, 750, {.maxSpeed = 80});

	chassis.moveToPoint(0, 0, 1250, {.forwards = false, .maxSpeed = 80}, false);
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	//claw.set_zero_position(-1000);


	while (true) {

		/*
		if (master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_R1)) {
            
            pros::lcd::set_text(6, "autonomo iniciado");
            
            pros::Task auto_task(autonomous);

            pros::delay(30000);

            auto_task.suspend();

            pros::lcd::set_text(6, "autonomo finalizado");
        }
			*/

		//2600
		//100
		int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);    // Gets amount forward/backward from left joystick
		int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		chassis.arcade(dir, turn); 

		/*
		
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && (std::abs(elevator.get_position()) <= 5100) ) {  // If R1 is pressed and elevator is not at top position
			elevator.move(127);                      // Move elevator up at full speed
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && (std::abs(elevator.get_position()) >= 200) ) {  // If L2 is pressed and elevator is not at bottom position
			elevator.move(-127);                     // Move elevator down at full speed
		} else {
			elevator.move(0);                        
		}

		*/

		/*

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			//wrist.move(127);
			wrist.move_absolute(740, 600);             
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {  
			wrist.move_absolute(0, 600);
			//wrist.move(-127);          
		}
			*/

			/*
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			//claw.move(127);
			claw.move_absolute(-1000, 100);              
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {  
			//claw.move(-127);
			claw.move_absolute(0, 100);               
		}
			*/

		//Elevator Max: 6000

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			piston.extend();        
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {  
			piston.retract();         
		}

		

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && (std::abs(elevator.get_position()) <= 3000) ) {  // If R1 is pressed and elevator is not at top position
			elevator.move(100);                      // Move elevator up at full speed
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && (std::abs(elevator.get_position()) >= 250) ) {  // If L2 is pressed and elevator is not at bottom position
			elevator.move(-100);                     // Move elevator down at full speed
		} else {
			elevator.move(0);                        
		}

		
		/*
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) == false) {
			setSelection(true, false);
		}

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
			goToGoal();

			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
				setSelection(false, false);
			}
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
				setSelection(false, true);
			}
		}
			*/
			

		pros::delay(20);	
	}
}