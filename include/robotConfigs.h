#ifndef _PROS_ROBOT_CONFIGS_H_
#define _PROS_ROBOT_CONFIGS_H_
#include "main.h"
#include "lemlib/api.hpp"
#include "pros/optical.hpp"

extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;
extern pros::Rotation horizontal_encoder;
extern pros::Rotation vertical_encoder;
extern lemlib::Chassis chassis;

extern pros::Controller master;

extern pros::Optical sensor;

extern pros::adi::Pneumatics piston;

extern pros::adi::Pneumatics wrist;

extern pros::MotorGroup elevator;

extern pros::Motor toggle;


extern pros::AIVision vision;

extern std::vector<lemlib::Pose> PedestalPoses;

#endif