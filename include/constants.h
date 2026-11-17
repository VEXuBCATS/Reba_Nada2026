#ifndef _PROS_CONSTANTS_H_
#define _PROS_CONSTANTS_H_
#include "main.h"
#include "robotConfigs.h"

extern int maxHeight;
extern int minHeight;
extern int trackingWheelDiameter; // in inches
extern lemlib::Pose InitialPose;
extern std::vector<lemlib::Pose> GoalPoses;
extern AllianceColor currentAlliance;
#endif