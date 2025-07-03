#include "robot.h"
#include "localisation/odometry.h"
#include "FSM.h"

using namespace vex;
using namespace std;

fsm::fsm(){
    State a;
}

void randomMovement(){
    // int i = rand(1,3);
    // if(i == 1){
    //     leftDrive.spin(forward, 50, percent);
    //     rightDrive.spin(forward, 50, percent);
}
void fsm::checkDist(){

}

void fsm::goBack(){

}

void fsm::updateState(fsm::State event){
    if(event == fsm::State::START){

    } else if(event == fsm::State::ROAM){

    } else if(event == fsm::State::CLEARING){

    } else if(event == fsm::State::RETURNING){

    } else if(event == fsm::State::STOP){

    }
}

void fsm::updateEvents(fsm::Transition event){
    if(event == fsm::Transition::FOUND){

    } else if(event == fsm::Transition::CLEARED){

    } else if(event == fsm::Transition::STARTREACHED){

    } else if(event == fsm::Transition::NOTRASH){

    }
}