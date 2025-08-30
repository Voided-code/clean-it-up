#ifndef FSM_H
#define FSM_H

#include "robot.h"
#include "localisation/odometry.h"
#include "vex.h"

using namespace vex;
using namespace std;

class fsm{
    public:
    enum State{
        START,
        ROAM,
        CLEARING,
        RETURNING,
        STOP
    };
    enum Transition{
        FOUND,
        CLEARED,
        STARTREACHED,
        NOTRASH
    };

    State a;
    Transition trans;

    fsm();
    void randomMovement(motor& leftDrive,motor& rightDrive);
     inline float clampf(float v, float lo, float hi);
     inline float normDeg(float a);
     inline float angleDiff(float target, float current);

    void checkDist();
    bool checkDist(float currRad, float targetRad);
    bool isobstacle(vision& Vision, vision::signature& Vision__SIG_1);
    void goTowards(motor& leftDrive,motor& rightDrive, vision& Vision, vision::signature& Vision__SIG_1);
    void goBack(motor& leftDrive,motor& rightDrive,robot& ror); //include the robot parameter (Robot& ror)
    void updateState(fsm::State event, robot& ror, motor& leftDrive,motor& rightDrive);
    void updateEvents(fsm::Transition transition, fsm::State state, robot& ror, motor& leftDrive,motor& rightDrive);

    private:


};

#endif
