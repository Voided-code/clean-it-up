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
    
    //first update radius each loop
    //second print out the radius on the odometer
    float radius = 0;
    while (true) {
        
        radius = sqrt(pow(currPos.first, 2) + pow(currPos.second, 2));
        ror->getOdom().update();
        currPos = {ror->getOdom().getX(),ror->getOdom().getY()};
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1,1);    
        Brain.Screen.print("X: %.1f Y: %.1f", currPos.first, currPos.second);
        Brain.Screen.setCursor(2,1);   
        Brain.Screen.print("Rad: %.1f", radius);
        tst->randomMovement(ror->getleftDrive(), ror->getrightDrive());
        if(tst->isobstacle(ror->Vision, ror->getVisionSignature())){
            break;
        }
        wait(200, msec);
    }
    delete ror;
}