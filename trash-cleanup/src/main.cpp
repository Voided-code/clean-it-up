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
    for (int i = 0; i < 50; i++) {
        printf("\n");
    }

    robot* ror = new robot();
    fsm* tst = new fsm();
    ror->start();
    ror->initialize();
    pair<float,float> startPos = {0,0};
    pair<float,float> currPos = {0,0};
    vex::brain screenRef = ror->getBrain();
    // function(int& something)
    //pointer* == points to an object (pointer cotains address)
    //reference& == referencing to that object (it does not reference to pointer)
    //we must dereference the pointer so that it will become object (*object)
    //main() int* x = 10; function(*x)  (dereference, converting pointer back to object)
    //first update radius each loop
    //second print out the radius on the odometer
    float radius = 0;
    tst->a= fsm::State::START;
    while (true) {
        
        // radius = sqrt(pow(currPos.first, 2) + pow(currPos.second, 2));
        ror->getOdom().update();
        // currPos = {ror->getOdom().getX(),ror->getOdom().getY()};
        // Brain.Screen.clearScreen();
        // Brain.Screen.setCursor(1,1);    
        // Brain.Screen.print("X: %.1f Y: %.1f", currPos.first, currPos.second);
        // Brain.Screen.setCursor(2,1);   
        // Brain.Screen.print("Rad: %.1f", radius);
        tst->updateState(tst->a,*ror,ror->getleftDrive(),ror->getrightDrive());
        // if(tst->isobstacle(ror->Vision, ror->getVisionSignature())){
        //     // break;
        //     tst->goTowards(ror->getleftDrive(), ror->getrightDrive(), ror->Vision, ror->getVisionSignature());
        // }
        // bool test= tst->isobstacle(ror->Vision, ror->getVisionSignature());
        // if(test){
        //     Brain.Screen.print("t");
        // } else {
        //     Brain.Screen.print("f");
        // }

        // tst->randomMovement(ror->getleftDrive(), ror->getrightDrive());
        // wait(200, msec);
    }
    delete ror;
    delete tst;
}