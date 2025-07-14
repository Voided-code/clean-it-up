#include "robot.h"
#include "localisation/odometry.h"
#include "FSM.h"
#include "vex.h"
#include <cstdlib>
#include <ctime>

using namespace vex;
using namespace std;

fsm::fsm(){
    State a;
}

void fsm::randomMovement(motor& leftDrive,motor& rightDrive) {

    // Assume your motors are defined as 'leftMotor' and 'rightMotor'

    // Seed the random number generator
    srand(time(NULL));

   
    // Generate random speed between -100 and 100
    int randomSpeed = (rand() % 201) - 100; 

    // Generate random duration (e.g., 500ms to 2000ms)
    int randomDuration = 500 + (rand() % 1501); 

    // Randomly choose a movement type
    int movementType = rand() % 5; // 0: forward/backward, 1: turn left, 2: turn right

    leftDrive.setVelocity(randomSpeed, percent);
    rightDrive.setVelocity(randomSpeed, percent);
    if (movementType <= 3) {
        leftDrive.spin(vex::forward);
        rightDrive.spin(vex::forward);
    } else if (movementType == 4) {
        leftDrive.spin(vex::forward);
        rightDrive.spin(vex::reverse);
    } else {
        leftDrive.spin(vex::reverse);
        rightDrive.spin(vex::forward);
    }
    printf("FSM left motor addr: %p\n", &leftDrive);

    wait(randomDuration, msec); // Wait for the random duration

    // Stop motors after movement
    leftDrive.stop();
    rightDrive.stop();
    wait(200, msec); // Short pause before next movement
    
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