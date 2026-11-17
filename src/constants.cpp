#include "robotConfigs.h"
#include "main.h"
#include <any>

static int maxHeight = 4050;
static int minHeight = 15;

static int trackingWheelDiameter = 2; // in inches

static lemlib::Pose InitialPose(0,-64, 0);
std::vector<lemlib::Pose> GoalPoses = {
    lemlib::Pose(0, 0),
    lemlib::Pose(-47, 23.5),
    lemlib::Pose(-47, -23.5),
    lemlib::Pose(-23.5, -47),
    lemlib::Pose(23.5, -47)
};


static AllianceColor currentAlliance = RED;