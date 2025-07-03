#ifndef FSM_H
#define FSM_H

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
    void randomMovement();
    void checkDist();
    void goBack();
    void updateState(fsm::State event);
    void updateEvents(fsm::Transition event);

    private:
    
     

};

#endif
