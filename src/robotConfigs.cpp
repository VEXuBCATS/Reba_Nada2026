#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pose.hpp"
#include "liblvgl/llemu.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/ai_vision.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include "robotConfigs.h"
#include "constants.h"
#include "autos.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::adi::Pneumatics piston('A', false, true); // extended_is_low = true -> lógica del pistón invertida

pros::MotorGroup right_motor_group({1, 2, -9}, pros::v5::MotorGears::blue);
// right motor group
pros::MotorGroup left_motor_group({-11, -12, 19}, pros::v5::MotorGears::blue);

pros::MotorGroup elevator({-15, -8}, pros::v5::MotorGears::green);

pros::Motor wrist(16, pros::v5::MotorGears::blue);

pros::Motor claw(7, pros::v5::MotorGears::red); 

pros::Motor toggle(17);

pros::Motor motor1(10, pros::v5::MotorGears::red);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group (swapped to invert turn)
                              &right_motor_group, // right motor group (swapped to invert turn)
                              15, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 10N (for now)
);

// imu
pros::Imu imu(20);
// horizontal tracking wheel encoder
pros::v5::AIVision vision(14);

pros::Rotation horizontal_encoder(18);
// vertical tracking wheel encoder
pros::Rotation vertical_encoder(-4);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -7.5);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 3.5);

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(12.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              38, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2.75, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              25, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              3 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

pros::Optical sensor(13);

int poseSelection = 0;

bool isToggleRED() {
    return sensor.get_hue() > 345 || sensor.get_hue() < 15;
}

bool isToggleBLUE() {
    return sensor.get_hue() > 180 && sensor.get_hue() < 300;
}

bool isToggleYELLOW() {
    return sensor.get_hue() > 30 && sensor.get_hue() < 80;
}

int calculateNearGoal() {
    double distance[5];
    int PedestalDesired = 0;
    for (int i = 0; i <= 4; i++) {
        distance[i] = std::hypot(chassis.getPose().x - GoalPoses[i].x, chassis.getPose().y - GoalPoses[i].y);
    }

    if (distance[0] < distance[1] && distance[0] < distance[2] && distance[0] < distance[3] && distance[0] < distance[4]) {
        PedestalDesired = 0;
    } if (distance[1] < distance[0] && distance[1] < distance[2] && distance[1] < distance[3] && distance[1] < distance[4]) {
        PedestalDesired = 1;
    } if (distance[2] < distance[0] && distance[2] < distance[1] && distance[2] < distance[3] && distance[2] < distance[4]) {
        PedestalDesired = 2;
    } if (distance[3] < distance[1] && distance[3] < distance[2] && distance[3] < distance[0] && distance[3] < distance[4]) {
        PedestalDesired = 3;
    } if (distance[4] < distance[1] && distance[4] < distance[2] && distance[4] < distance[0] && distance[4] < distance[3]) {
        PedestalDesired = 4;
    }

    return PedestalDesired;
}

void goToGoal() {
    if (poseSelection < 0) {
        poseSelection = 0;
    } if (poseSelection > 4) {
        poseSelection = 4;
    }
    chassis.moveToPoint(GoalPoses[poseSelection].x, GoalPoses[poseSelection].y, 50, {.maxSpeed = 50});
}

void setLevelElevator(int level) {
    if (level == 0) {
        elevator.move_absolute(15, 200);
    } else if (level == 1) {
        if (calculateNearGoal() == 0) {
            elevator.move_absolute(1700, 200); //
        } else if (calculateNearGoal() == 1 || calculateNearGoal() == 4) {
            elevator.move_absolute(1230, 200); //
        } else if (calculateNearGoal() == 2 || calculateNearGoal() == 3) {
            elevator.move_absolute(750, 200); //
        }
    } else if (level == 2) {
        if (calculateNearGoal() == 0) {
            elevator.move_absolute(2600, 200); //
        } else if (calculateNearGoal() == 1 || calculateNearGoal() == 4) {
            elevator.move_absolute(2200, 200); //
        } else if (calculateNearGoal() == 2 || calculateNearGoal() == 3) {
            elevator.move_absolute(1750, 200); //
        }
    } else if (level == 3) {
        if (calculateNearGoal() == 0) {
            elevator.move_absolute(3600, 200); //
        } else if (calculateNearGoal() == 1 || calculateNearGoal() == 4) {
            elevator.move_absolute(3200, 200); //
        } else if (calculateNearGoal() == 2 || calculateNearGoal() == 3) {
            elevator.move_absolute(2820, 200); //
        }
    } else if (level == 4) {
        if (calculateNearGoal() == 0) {
            elevator.move_absolute(3600, 200); //
        } else if (calculateNearGoal() == 1 || calculateNearGoal() == 4) {
            elevator.move_absolute(4050, 200); //
        } else if (calculateNearGoal() == 2 || calculateNearGoal() == 3) {
            elevator.move_absolute(3800, 200); //
        }
    }
}

void setSelection(bool reset, bool increment) {
    if (reset) {
        poseSelection = calculateNearGoal();
    } else if (increment) {
        poseSelection = poseSelection + 1;
    } else if (!increment) {
        poseSelection = poseSelection - 1;
    }
}


pros::Task screen_task([]() {
    
    while (true) {
        pros::aivision_object_s_t detected_object = vision.get_object(0);

        //lineas 2 y 3 disponibles

        //print match and auto configs
        pros::lcd::print(0, "Alliance: %s ", std::string((currentAlliance == RED) ? "RED" : "BLUE"));
        pros::lcd::print(1, "Autonomo: %s ", AutoModeNames[static_cast<int>(autoMode)]);
        // print robot location to the brain screen
        pros::lcd::print(4, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(5, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(6, "Theta: %f", chassis.getPose().theta);
        //pros::lcd::print(2, "Elevator: %f", elevator.get_position()); // elevator position
        //pros::lcd::set_text(3, "Claw Position: " + std::to_string(claw.get_position())); // wrist position
        pros::lcd::print(7, "Goal: %d", calculateNearGoal());
        
        // delay to save resources
        pros::delay(50);
    }
});

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void runAuto() {
    for (int i = 3; i >= 1; i--) {
        pros::lcd::clear();
        pros::lcd::print(3, "Auto en: %d", i);
        pros::delay(1000);
    }

    pros::lcd::set_text(6, "autonomo iniciado");
            
    pros::Task auto_task(autonomous);

    pros::delay(30000);

    auto_task.suspend();

    pros::lcd::set_text(6, "autonomo finalizado");
}

void changeAlliance() {
    currentAlliance = (currentAlliance == RED) ? BLUE : RED;
}

void changeAuto() {
    autoMode = static_cast<AutoMode>((static_cast<int>(autoMode) + 1) % TOTAL);
}

// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    chassis.setPose(InitialPose.x, InitialPose.y, InitialPose.theta);

    pros::Motor(15).set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::Motor(8).set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    pros::Motor(15).brake();
    pros::Motor(8).brake();
    elevator.tare_position();

    setLevelElevator(0);	

    pros::lcd::register_btn0_cb(changeAlliance);
	pros::lcd::register_btn1_cb(on_center_button);
    pros::lcd::register_btn2_cb(changeAuto);
}