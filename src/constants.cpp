#include "robotConfigs.h"
#include "constants.h"
#include "main.h"
#include <any>

int maxHeight = 4050;
int minHeight = 15;

int trackingWheelDiameter = 2; // in inches

lemlib::Pose InitialPose(0,-64, 0);
std::vector<lemlib::Pose> GoalPoses = {
    lemlib::Pose(0, 0),
    lemlib::Pose(-47, 23.5),
    lemlib::Pose(-47, -23.5),
    lemlib::Pose(-23.5, -47),
    lemlib::Pose(23.5, -47)
};


AllianceColor currentAlliance = RED;