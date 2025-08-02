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

   
    // Generate random speed between -50 and 50
    int randomSpeed = (rand() % 101) - 50; 

    // Generate random duration (e.g., 500ms to 2000ms)
    int randomDuration = 500 + (rand() % 1501); 

    // Randomly choose a movement type
    int movementType = rand() % 4; // 0: forward/backward, 1: turn left, 2: turn right

    leftDrive.setVelocity(randomSpeed, percent);
    rightDrive.setVelocity(randomSpeed, percent);
    if (movementType <= 2) {
        leftDrive.spin(vex::forward);
        rightDrive.spin(vex::forward);
    } else if (movementType == 3) {
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
bool fsm::checkDist(float currRad, float targetRad){
    if(currRad > targetRad){
        return false;
    }
    return true;
}

bool fsm::isobstacle(vision& Vision, vision::signature& Vision__SIG_1){
    Vision.takeSnapshot(Vision__SIG_1);
    if(Vision.objectCount > 0){
        return true;
    } else
        return false;
}

void fsm::goTowards(motor& leftDrive,motor& rightDrive, vision& Vision, vision::signature& Vision__SIG_1){
    leftDrive.setStopping(hold);
    rightDrive.setStopping(hold);
    leftDrive.setVelocity(40, percent);
    rightDrive.setVelocity(40, percent);
    int hold = 0;
    
    // if(Vision.objects[1].centerX < 315/2){
    //     while(Vision.objects[1].centerX < 315/2){
    //         leftDrive.setVelocity(40 /* placeholder value */, percent);
    //         rightDrive.setVelocity(40 /* placeholder value */, percent);
    //         leftDrive.spin(vex::forward);
    //         rightDrive.spin(vex::reverse);
    //         wait(20, msec);
    //     }
    //     leftDrive.stop();
    //     rightDrive.stop();
    // } else{
    //     while(Vision.objects[1].centerX > 315/2){
    //         leftDrive.setVelocity((Vision.objects[1].centerX-157.5)/2, percent);
    //         rightDrive.setVelocity((Vision.objects[1].centerX-157.5)/2, percent);
    //         leftDrive.spin(vex::forward);
    //         rightDrive.spin(vex::reverse);
    //         wait(20, msec);
    //     }
    //     leftDrive.stop();
    //     rightDrive.stop();
    // }

    leftDrive.spin(vex::forward);
    rightDrive.spin(vex::forward);
    //TODO: good job in the future maybe put in isobstacle
    while(hold < 20){
        Vision.takeSnapshot(Vision__SIG_1);
        while (Vision.objectCount > 0) {
            hold = 0;
            Vision.takeSnapshot(Vision__SIG_1);
            wait(50, msec);
        }
        hold++;
        wait(50, msec);
    }
}

void fsm::goBack(){

}

void fsm::updateState(fsm::State event){
    if(event == fsm::State::START){

    } else if(event == fsm::State::ROAM){
        /** if see trash then updateEvent(transition::Foundtrash,currentstate) */

    } else if(event == fsm::State::CLEARING){

    } else if(event == fsm::State::RETURNING){

    } else if(event == fsm::State::STOP){

    }
}
//todo: write update event
void fsm::updateEvents(fsm::Transition transition, fsm::State state){
    if(transition == fsm::Transition::FOUND){
        if(state==fsm::ROAM){
            updateState(fsm::CLEARING);
        }    

    } else if(transition == fsm::Transition::CLEARED){
        if(state==fsm::CLEARING){
            updateState(fsm::RETURNING);
        }    

    } else if(transition == fsm::Transition::STARTREACHED){
        if(state==fsm::RETURNING){
            updateState(fsm::ROAM);
        }    

    } else if(transition == fsm::Transition::NOTRASH){
        if(state==fsm::ROAM){
            updateState(fsm::STOP);
        }   
    }
}