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

    fsm();
    void randomMovement(motor& leftDrive, motor& rightDrive);
    void checkDist();
    bool checkDist(float currRad, float targetRad);
    bool isobstacle(vision& Vision, vision::signature& Vision__SIG_1);
    void goTowards(motor& leftDrive,motor& rightDrive, vision& Vision, vision::signature& Vision__SIG_1);
    void goBack();
    void updateState(fsm::State event);
    void updateEvents(fsm::Transition transition, fsm::State state);

    private:


};

#endif
