#include "localisation/odometry.h"
#include "vex.h"

using namespace vex;

Odometry::Odometry(motor &left, motor &right, inertial &imu)
    : leftMotor(left), rightMotor(right), imuSensor(imu)
{
    robotX = 0.0;
    robotY = 0.0;
    robotHeading = 0.0;
    prevLeftEncoder = 0.0;
    prevRightEncoder = 0.0;
    isCalibrated = false;
}

void Odometry::initialize()
{
    imuSensor.calibrate();

    while (imuSensor.isCalibrating())
    {
        wait(50, msec);
    }

    leftMotor.resetPosition();
    rightMotor.resetPosition();

    prevLeftEncoder = 0.0;
    prevRightEncoder = 0.0;

    robotX = 0.0;
    robotY = 0.0;
    robotHeading = 0.0;

    isCalibrated = true;
}

void Odometry::setPosition(float x, float y, float heading)
{
    robotX = x;
    robotY = y;
    robotHeading = heading;
}

void Odometry::update()
{
    if (!isCalibrated)
        return;

    float currentLeftEncoder = leftMotor.position(degrees);
    float currentRightEncoder = rightMotor.position(degrees);
   
    float deltaLeftEncoder = currentLeftEncoder - prevLeftEncoder;
    float deltaRightEncoder = currentRightEncoder - prevRightEncoder;

    float leftDistance = degreesToDistance(deltaLeftEncoder);
    float rightDistance = degreesToDistance(deltaRightEncoder);

    float deltaDistance = (leftDistance + rightDistance) / 2.0;

    robotHeading = imuSensor.heading(degrees);

    float headingRadians = -robotHeading * M_PI / 180.0;
    robotX += deltaDistance * cos(headingRadians);
    robotY += deltaDistance * sin(headingRadians);

    prevLeftEncoder = currentLeftEncoder;
    prevRightEncoder = currentRightEncoder;
}

float Odometry::getX()
{
    return robotX;
}

float Odometry::getY()
{
    return robotY;
}

float Odometry::getHeading()
{
    return robotHeading;
}

float Odometry::getDistanceTo(float targetX, float targetY)
{
    float dx = targetX - robotX;
    float dy = targetY - robotY;
    return sqrt(dx * dx + dy * dy);
}

float Odometry::getAngleTo(float targetX, float targetY)
{
    float dx = targetX - robotX;
    float dy = targetY - robotY;
    return atan2(dy, dx) * 180.0 / M_PI;
}

void Odometry::displayOnScreen(vex::brain::lcd& screen)
{
    screen.clearScreen();
    screen.setCursor(1, 1);
    screen.print("X: %.1fmm", robotX);

    screen.setCursor(2, 1);
    screen.print("Y: %.1fmm", robotY);

    screen.setCursor(3, 1);
    screen.print("H: %.1f°", robotHeading);

    screen.setCursor(4, 1);
    screen.print("L: %.0f R: %.0f",
                       leftMotor.position(degrees),
                       rightMotor.position(degrees));
}

void Odometry::reset()
{
    robotX = 0.0;
    robotY = 0.0;
    robotHeading = 0.0;
    leftMotor.resetPosition();
    rightMotor.resetPosition();
    prevLeftEncoder = 0.0;
    prevRightEncoder = 0.0;
}

float Odometry::degreesToDistance(float degrees)
{
    float rotations = degrees / 360.0;
    float distance = rotations * M_PI * WHEEL_DIAMETER * GEAR_RATIO;
    return distance;
}