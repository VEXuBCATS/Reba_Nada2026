#ifndef _PROS_CONSTANTS_H_
#define _PROS_CONSTANTS_H_
#include "main.h"
#include "robotConfigs.h"

enum AllianceColor {
    RED,
    BLUE, 
    any
};

extern int maxHeight;
extern int minHeight;
extern int minWristPosition;
extern int trackingWheelDiameter; // in inches
extern lemlib::Pose InitialPose;
extern std::vector<lemlib::Pose> GoalPoses;
extern AllianceColor currentAlliance;

extern bool isTaking;
extern bool isJoystick;
#endif