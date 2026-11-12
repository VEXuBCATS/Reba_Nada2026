#ifndef _PROS_ROBOT_CONFIGS_H_
#define _PROS_ROBOT_CONFIGS_H_
#include "main.h"
#include "lemlib/api.hpp"

extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;
extern lemlib::Chassis chassis;

extern pros::adi::Pneumatics piston;

extern pros::MotorGroup elevator;

extern pros::Motor wrist;

extern pros::Motor claw;

extern pros::Motor toggle;

extern pros::Motor motor;

extern pros::AIVision vision;

extern std::vector<lemlib::Pose> PedestalPoses;

#endif