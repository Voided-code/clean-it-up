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
      inertial imu = inertial();
      Odometry* odom;

    public:
        robot();
        void start();
        void initialize();
        void updateTick();
        void display();

        motor& getleftDrive();
        motor& getrightDrive();
        brain& getBrain();
        Odometry& getOdom();
};
//the guard is here i cant speak, can you rerun the code i found the issue, try to re run the code are u here
//hello alexander please re run the code
//yep let me fix gimme a sec 
//rerun it now.(again)
#endif