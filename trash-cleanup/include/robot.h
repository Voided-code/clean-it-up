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
      motor leftDrive=motor(PORT1, true);
      motor rightDrive=motor(PORT7, false);
      vision::signature Vision__SIG_1 = vision::signature (1, -4131, -3023, -3577,6085, 8193, 7139,2, 0);
      inertial imu = inertial();
      Odometry* odom;

    public:
        vex::vision Vision = vex::vision(PORT12, 150, Vision__SIG_1);

        robot();
        void start();
        void initialize();
        void updateTick();
        void display();
        
        vision::signature& getVisionSignature();
        motor& getleftDrive();
        motor& getrightDrive();
        brain& getBrain();
        Odometry& getOdom();
};
#endif