#include "main.h"
//#include "pros/abstract_motor.hpp"
//#include "pros/imu.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "robotConfigs.h"
#include "constants.h"
#include "autos.h"


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

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
void competition_initialize() {
}

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
	switch (autoMode) {
		case AutoMode::auto1:
			auto1();
			break;
		case AutoMode::auto2:
			auto2();
			break;
		case AutoMode::autoTest:
			autoTest();
			break;
		case AutoMode::autoOffset:
			autoOffset();
			break;
		default:
			break;
	}
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
	int level = 0;
	uint32_t tiempoInicio = 0;;
	bool isExecute = false;
	while (true) {

		int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);    // Gets amount forward/backward from left joystick
		int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		chassis.arcade(dir, turn); 

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
			elevator.move_velocity(100);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			elevator.move_velocity(-100);
		} else {
			elevator.move_velocity(0);
		}
		
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			sensor.set_led_pwm(100);
			if ((currentAlliance == RED) ? isToggleRED() : isToggleBLUE()) {
				toggle.move_velocity(0);
			} else {
				toggle.move_velocity(200);
			}	
		} else {
			sensor.set_led_pwm(0);
			toggle.move_velocity(0);
		}	

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			pros::Task wristTask(turnWrist);
		}
		
		if (master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) >= 120) {
			if (!isExecute) {
				level += 1;
				if (level > 4) {
					level = 4;
				}
				isExecute = true;
			}
		} else if (master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) <= -120) {
			if (!isExecute) {
				level -= 1;
				if (level < 0) {
					level = 0;
				}
				isExecute = true;
			}
		} else {
			isExecute = false;
		}
			

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			if (!wrist.is_extended()) {
				setLevelElevator(0);
			} else {
				pros::Task wristTask(turnWrist);
			}
			piston.extend(); 
			isTaking = true;       
			level = 1;
		} else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			piston.retract();
			if (isTaking) {
				tiempoInicio = pros::millis();
			}
			isTaking = false;
		}

		/*

		if(!master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && (pros::millis() - tiempoInicio) >= 250 && !master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			setLevelElevator(level);
		}

		*/

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			piston.extend();
			elevator.move_relative(-50, 200);     
		}	

		pros::delay(20);	
	}
}