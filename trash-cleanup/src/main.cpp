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

using namespace vex;

int main() {
    robot* ror = new robot();
    fsm* tst = new fsm();
    ror->start();
    ror->initialize();
    while (true) {
        ror->updateTick();
        wait(20, msec);
    }
    delete ror;
}