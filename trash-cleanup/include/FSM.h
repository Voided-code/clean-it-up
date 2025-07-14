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
    void goBack();
    void updateState(fsm::State event);
    void updateEvents(fsm::Transition event);

    private:


};

#endif
