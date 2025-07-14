#include "localisation/odometry.h"
#include "vex.h"
#include "robot.h"

using namespace vex;
using namespace std;

robot::robot(){
}

void robot::start(){
    odom=new Odometry(leftDrive, rightDrive, imu);
}

void robot::initialize(){
    odom->initialize();
}

void robot::updateTick(){
    odom->update();
}

motor& robot::getleftDrive(){
    return leftDrive;
}

motor& robot::getrightDrive(){
    return rightDrive;
}

brain& robot::getBrain(){
    return Brain;
}

Odometry& robot::getOdom(){
    return *odom;
}