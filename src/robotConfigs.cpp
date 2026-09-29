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
//#include "commands/CommandScheduler.hpp"

lemlib::Pose InitialPose(0,-64, 0);

pros::adi::Pneumatics piston('A', false);

pros::MotorGroup right_motor_group({1, 2, -9}, pros::v5::MotorGears::blue);
// right motor group
pros::MotorGroup left_motor_group({-11, -12, 19}, pros::v5::MotorGears::blue);

pros::MotorGroup elevator({15, -8}, pros::v5::MotorGears::green);

pros::Motor wrist(16, pros::v5::MotorGears::blue);

pros::Motor claw(7, pros::v5::MotorGears::red); 

pros::Motor motor1(10, pros::v5::MotorGears::red);

int poseSelection = 0;

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group (swapped to invert turn)
                              &right_motor_group, // right motor group (swapped to invert turn)
                              15, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              320, // drivetrain rpm is 600
                              2 // horizontal drift is 10N (for now)
);

// imu
pros::Imu imu(20);
// horizontal tracking wheel encoder
pros::v5::AIVision vision(21);


pros::Rotation horizontal_encoder(4);
// vertical tracking wheel encoder
pros::Rotation vertical_encoder(-18);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -7.25);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -2.75);

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

std::vector<lemlib::Pose> GoalPoses = {
    lemlib::Pose(0, 0),
    lemlib::Pose(-47, 23.5),
    lemlib::Pose(-47, -23.5),
    lemlib::Pose(-23.5, -47),
    lemlib::Pose(23.5, -47)
};

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
            elevator.move_absolute(2200, 200); //1400
        } else if (calculateNearGoal() == 1 || calculateNearGoal() == 4) {
            elevator.move_absolute(1860, 200); //960
        } else if (calculateNearGoal() == 2 || calculateNearGoal() == 3) {
            elevator.move_absolute(1460, 200); //560
        }
    } else if (level == 2) {
        if (calculateNearGoal() == 0) {
            elevator.move_absolute(3400, 200); //2500
        } else if (calculateNearGoal() == 1 || calculateNearGoal() == 4) {
            elevator.move_absolute(3000, 200); //2100
        } else if (calculateNearGoal() == 2 || calculateNearGoal() == 3) {
            elevator.move_absolute(2600, 200); //1700
        }
    } else if (level == 3) {
        if (calculateNearGoal() == 0) {
            elevator.move_absolute(4600, 200); //3700
        } else if (calculateNearGoal() == 1 || calculateNearGoal() == 4) {
            elevator.move_absolute(4100, 200); //3200
        } else if (calculateNearGoal() == 2 || calculateNearGoal() == 3) {
            elevator.move_absolute(3700, 200); //2800
        }
    } else if (level == 4) {
        if (calculateNearGoal() == 0) {
            elevator.move_absolute(4600, 200); //3700
        } else if (calculateNearGoal() == 1 || calculateNearGoal() == 4) {
            elevator.move_absolute(4600, 200); //4400
        } else if (calculateNearGoal() == 2 || calculateNearGoal() == 3) {
            elevator.move_absolute(4600, 200); //4000
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
        vision.enable_detection_types(pros::AivisionModeType::tags);

        // print robot location to the brain screen
        pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
        pros::lcd::print(4, "Elevator: %f", elevator.get_position()); // elevator position
        //pros::lcd::set_text(5, "Claw Position: " + std::to_string(claw.get_position())); // wrist position
        pros::lcd::print(5, "Goal: %d", calculateNearGoal());

        if (pros::AIVision::is_type(vision.get_object(0), pros::AivisionDetectType::tag)) {
            pros::lcd::print(6, "Id Detected: %d", vision.get_object(0).id);
        } else {
            pros::lcd::print(6, "No Id detected");
        }
        
		//pros::lcd::set_text(3, "Wrist Position: " + std::to_string(wrist.get_position()));

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

// initialize function. Runs on program startup
void initialize() {
    //scheduler.run();

    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    chassis.setPose(InitialPose.x, InitialPose.y, InitialPose.theta);
    // La tarea de pantalla ya está corriendo en segundo plano

    //pros::lcd::set_text(1, "Hello PROS User!");

    pros::Motor(15).set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::Motor(8).set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    pros::Motor(15).brake();
    pros::Motor(8).brake();
    elevator.tare_position();
    setLevelElevator(0);

	

	//pros::lcd::register_btn1_cb(on_center_button);
}