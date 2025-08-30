#include "robot.h"
#include "localisation/odometry.h"
#include "FSM.h"
#include "vex.h"
#include <cstdlib>
#include <ctime>

using namespace vex;
using namespace std;

fsm::fsm(){
}
 inline float fsm::clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

// Normalize angle into [-180, 180)
 inline float fsm::normDeg(float a) {
    while (a >= 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// Compute smallest signed difference target - current in [-180, 180)
 inline float fsm::angleDiff(float target, float current) {
    return normDeg(target - current);
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
    int movementType = rand() % 5; // 0: forward/backward, 1: turn left, 2: turn right

    leftDrive.setVelocity(randomSpeed, percent);
    rightDrive.setVelocity(randomSpeed, percent);
    if (movementType == 1) {
        leftDrive.spin(vex::forward);
        rightDrive.spin(vex::forward);
    } else if (movementType == 2 || movementType == 3) {
        leftDrive.spin(vex::forward);
        rightDrive.spin(vex::reverse);
    } else {
        leftDrive.spin(vex::reverse);
        rightDrive.spin(vex::forward);
    }

    wait(randomDuration, msec);
    // int randomDurationi = 0;
    // while (randomDurationi < randomDuration) {
    //     wait(100, msec);
    //     randomDurationi += 100;
    //     if(isobstacle(ror.Vision, ror.getVisionSignature()))
    //         break;
    // }

    // Stop motors after movement
    leftDrive.stop();
    rightDrive.stop();
    wait(200, msec); // Short pause before next movement
    
}

bool fsm::checkDist(float currRad, float targetRad){
    if(currRad > targetRad){
        return true;
    }
    return false;
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

    int time = 0;
    leftDrive.spin(vex::forward);
    rightDrive.spin(vex::forward);
    //TODO: good job in the future maybe put in isobstacle
    while(hold < 8 && time < 50) {
        Vision.takeSnapshot(Vision__SIG_1);
        while (Vision.objectCount > 0) {
            time++;
            hold = 0;
            Vision.takeSnapshot(Vision__SIG_1);
            wait(20, msec);
        }
        time++;
        hold++;
        wait(20, msec);
    }
}
//TODO: NEED TO FIX (SOME AI GENERATED FIX HEHE)
void fsm::goBack(vex::motor& leftDrive, vex::motor& rightDrive, robot& ror) {
    // --- knobs ---
    const float turnKp   = 0.7f;   // proportional turn gain
    const float turnMin  = 6.0f;   // % to break static friction
    const float turnMax  = 55.0f;  // % clamp
    const float turnTol  = 2.0f;   // deg tolerance to start driving
    const float drivePct = 50.0f;  // forward speed after alignment

    // If your IMU zero ≠ field +X, set an offset in CW degrees (else leave 0).
    const float HEADING_OFFSET_CW = 0.0f;

    // Vector to origin
    float dx = -ror.getOdom().getX();
    float dy = -ror.getOdom().getY();

    // Target heading: atan2 is CCW; convert to IMU CW
    float tgtCCW = std::atan2(dy, dx) * 57.2957795f;  // [-180,180]
    if (tgtCCW < 0.0f) tgtCCW += 360.0f;              // [0,360)
    float targetCW = 360.0f - tgtCCW;                 // CW
    if (targetCW >= 360.0f) targetCW -= 360.0f;

    // Current heading (CW) + optional offset so frames match
    inertial imu = ror.getIMU();
    float hdg = imu.heading(degrees) + HEADING_OFFSET_CW;
    while (hdg >= 360.0f) hdg -= 360.0f;
    while (hdg <    0.0f) hdg += 360.0f;

    // Shortest signed CW error in (-180,180]
    float err = targetCW - hdg;
    while (err <= -180.0f) err += 360.0f;
    while (err >   180.0f) err -= 360.0f;

    // Motor mode (set once is fine; cheap to set every tick as well)
    leftDrive.setStopping(brake);
    rightDrive.setStopping(brake);

    if (std::fabs(err) > turnTol) {
        // TURN IN PLACE this tick
        float cmd = turnKp * err;
        if (cmd >  turnMax) cmd =  turnMax;
        if (cmd < -turnMax) cmd = -turnMax;
        if (std::fabs(cmd) < turnMin) cmd = (cmd < 0.0f ? -turnMin : turnMin);

        // Positive cmd => CW (left fwd, right rev). Flip signs if wiring is opposite.
        leftDrive.setVelocity( cmd, percent);
        rightDrive.setVelocity(-cmd, percent);
        leftDrive.spin(forward);
        rightDrive.spin(forward);
    } else {
        // ALIGNED: DRIVE FORWARD (outer state will stop on distance)
        leftDrive.setVelocity(drivePct, percent);
        rightDrive.setVelocity(drivePct, percent);
        leftDrive.spin(forward);
        rightDrive.spin(forward);
    }
}


void fsm::updateState(fsm::State event, robot& ror, motor& leftDrive,motor& rightDrive){
    this->a=event;
    //here
    if(a == fsm::State::START){
        printf("state: start\n");
        updateState(fsm::State::ROAM,ror,leftDrive,rightDrive);
    } else if(a == fsm::State::ROAM){
        printf("state: roam\n");
         if(checkDist(sqrt(pow(ror.getOdom().getX(), 2) + pow(ror.getOdom().getY(), 2)), 300)){
            updateState(fsm::RETURNING, ror,leftDrive,rightDrive);
        }

        randomMovement(leftDrive,rightDrive);
        if(isobstacle(ror.Vision, ror.getVisionSignature())){
            updateEvents(fsm::Transition::FOUND, a, ror,leftDrive,rightDrive);
        }
    //gotowards placed top test
    //gotowards plced bottom test
    }else if(a == fsm::State::CLEARING){
        printf("state: clearing\n");
        if(checkDist(sqrt(pow(ror.getOdom().getX(), 2) + pow(ror.getOdom().getY(), 2)), 300))
            updateEvents(fsm::Transition::CLEARED, a, ror, leftDrive, rightDrive);
        // if (!isobstacle(ror.Vision, ror.getVisionSignature())) {
        //     updateEvents(fsm::Transition::CLEARED, a, ror, leftDrive, rightDrive);
        // }
        goTowards(ror.getleftDrive(), ror.getrightDrive(), ror.Vision, ror.getVisionSignature());
        bool PRINTVAL = checkDist(sqrt(pow(ror.getOdom().getX(), 2) + pow(ror.getOdom().getY(), 2)), 300);
        // ror.getBrain().Screen.clearScreen();
        // ror.getBrain().Screen.setCursor(1,1);
        // ror.getBrain().Screen.print("RadB: %d", PRINTVAL);
        // ror.getBrain().Screen.setCursor(2,1);
        // ror.getBrain().Screen.print("Dist: %f", sqrt(pow(ror.getOdom().getX(), 2) + pow(ror.getOdom().getY(), 2)));

    } else if(a == fsm::State::RETURNING){
        goBack(leftDrive,rightDrive,ror);
        if(sqrt(pow(ror.getOdom().getX(), 2) + pow(ror.getOdom().getY(), 2)) > 300){
            printf("state: returning\n");
            goBack(leftDrive,rightDrive,ror);
        }
        else{
            //switching to roaming
            updateEvents(fsm::Transition::STARTREACHED, a, ror,leftDrive,rightDrive);
        }
    } else if(a == fsm::State::STOP){
        printf("state: stop\n");
        //dont call this yet
    }
}

//todo: write update event
void fsm::updateEvents(fsm::Transition transition, fsm::State state, robot& ror, motor& leftDrive,motor& rightDrive){
    this->trans=transition;
    if(trans == fsm::Transition::FOUND){
        if(state==fsm::ROAM){
            updateState(fsm::CLEARING, ror,leftDrive,rightDrive);
        }    

    } else if(trans == fsm::Transition::CLEARED){
        if(state==fsm::CLEARING){
            updateState(fsm::RETURNING, ror,leftDrive,rightDrive);
        }    

    } else if(trans == fsm::Transition::STARTREACHED){
        if(state==fsm::RETURNING){
            updateState(fsm::ROAM, ror,leftDrive,rightDrive);
        }    

    } else if(trans == fsm::Transition::NOTRASH){
        if(state==fsm::ROAM){
            updateState(fsm::STOP, ror,leftDrive,rightDrive);
        }   
    }
}