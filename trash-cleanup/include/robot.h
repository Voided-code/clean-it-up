#ifndef ROBOT_H
#define ROBOT_H

#include "robot.h"
#include "vex.h"
#include "localisation/odometry.h"

using namespace vex;
using namespace std;

class robot{
    private:
      brain Brain;
      motor leftDrive = motor(PORT1, true);
      motor rightDrive = motor(PORT7, false);
      inertial imu = inertial();
      Odometry* odom;

    public:
        robot();
        void start();
        void initialize();
        void updateTick();

        motor getleftDrive();
        motor getrightDrive();
        brain getBrain();
        Odometry getOdom();
};

#endif