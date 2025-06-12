#pragma once
#include "pros/adi.hpp"
#include "pros/imu.hpp"

void odomInit();
void odomUpdate();
double get_x();
double get_y();
double get_heading_rad();
void coordinatePrint();

//movement functions
void moveToPoint(double targetX, double targetY);
void oldMoveToPoint(double targetX, double targetY);
void turnToHeading(double targetAngle);
void moveOneAxis(float targetY);
void moveToPose(double targetX, double targetY, double targetHeading, double radius, double speed, bool clockwise);