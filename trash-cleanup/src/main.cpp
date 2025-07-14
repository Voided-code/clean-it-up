/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       alexa                                                     */
/*    Created:      7/2/2025, 2:54:03 PM                                      */
/*    Description:  IQ2 project                                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "localisation/odometry.h"
#include "robot.h"
#include "FSM.h"
#include <utility>
#include <string>

using namespace vex;
using namespace std;

brain Brain;
// motor name = motor(PORT1, true);

int main() {
    robot* ror = new robot();
    fsm* tst = new fsm();
    ror->start();
    ror->initialize();
    pair<float,float> startPos = {0,0};
    pair<float,float> currPos = {0,0};
    vex::brain screenRef = ror->getBrain();
    while (true) {
        ror->getOdom().update();
        currPos = {ror->getOdom().getX(),ror->getOdom().getY()};
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1,1);    
        Brain.Screen.print("X: %.1f Y: %.1f", currPos.first, currPos.second);
        tst->randomMovement(ror->getleftDrive(), ror->getrightDrive());
        wait(200, msec);
    }
    delete ror;
}