#pragma once
#include "pros/adi.hpp"
#include "pros/imu.hpp"

void odomInit();
void odomUpdate();
double get_x();
double get_y();
double get_heading_rad();
void coordinatePrint();

void moveToPoint(double target_x, double target_y);
void turnToHeading(double targetAngle);
void moveToPose();
