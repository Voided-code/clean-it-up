#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "vex.h"
using namespace vex;

class Odometry
{
private:
    motor &leftMotor;
    motor &rightMotor;
    inertial &imuSensor;

    const float WHEEL_DIAMETER = 65.7;
    const float GEAR_RATIO = 2;

    float robotX;
    float robotY;
    float robotHeading;

    float prevLeftEncoder;
    float prevRightEncoder;

    bool isCalibrated;

    float degreesToDistance(float degrees);

public:
    Odometry(motor &left, motor &right, inertial &imu);

    void initialize();
    void setPosition(float x, float y, float heading);
    void update();

    float getX();
    float getY();
    float getHeading();
    float getDistanceTo(float targetX, float targetY);
    float getAngleTo(float targetX, float targetY);

    void displayOnScreen(vex::brain::lcd& screen);
    void reset();

    bool ready() const { return isCalibrated; }
};

#endif