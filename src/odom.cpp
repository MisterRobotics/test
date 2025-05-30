#include "odom.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <cmath>

//sensors
pros::Distance dist_right(9); // right distance sensor, tracks distance to right facing wall
pros::Distance dist_back(11);  // back distance sensor, tracks distance to back facing wall
pros::Imu imu(20);             //inertial sesnor, tracks heading

pros::Motor leftMotor(1, pros::MotorGearset::green);
pros::Motor rightMotor(-10, pros::MotorGearset::green);
//pros::MotorGroup left_drive({4,5,6}, pros::MotorGearset::blue);
//pros::MotorGroup right_drive({7,8,9}, pros::MotorGearset::blue);

//move to pose pure pursuit limits
const float lookaheadDistance = 6.0; // inches
const float maxSpeed = 115.0;        // percent voltage
const float kTurn = 1.5;             // heading correction gain
const float kDrive = 50;
const float posTolerance = 1;      // position tolerance in inches


//Speical functions
double clamp(double value, double minValue, double maxValue)
{
    return std::max(minValue, std::min(value, maxValue));
}

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

// Robot geometry
const double x_offset = 4.0;  // Right sensor offset from center (in) 
const double y_offset = 5.0;  // Back sensor offset from center (in)

double pos_x = 7.5; // robots X position on the field
double pos_y = 4; // robots Y position on the field
double heading = 0.0; // robots heading in radians

void odomInit() 
{
    //calibrate inertial sensor before running odom
  imu.reset();
  while (imu.is_calibrating()) pros::delay(20);
}

void odomUpdate() 
{
  // et robot heading in radians from inertial sesnor
  heading = imu.get_heading() * (M_PI / 180.0);

  // Convert sensor readings to inches (from mm)
  double right_dist = dist_right.get() / 25.4;
  double back_dist = dist_back.get() / 25.4;

  // Right wall is at X = 144, so subtract from that
  pos_x = 144.0 - (right_dist + x_offset * cos(heading));

  // Back wall is at Y = 0, so add distance from back wall
  pos_y = back_dist + y_offset * sin(heading);

}

void coordinatePrint()
{
    while(true)
    {
        pros::lcd::print(1, "X: %f", pos_x);
        pros::lcd::print(2, "Y: %f", pos_y);
        pros::lcd::print(3, "Theta: %f", heading);
        pros::delay(15);
    }
}

double get_x() { return pos_x; }
double get_y() { return pos_y; }
double get_heading_rad() { return heading; }

void driveRobot(float left, float right)
{
    //apply voltage of variables left and right to drive motors to spin them appropriatly
    leftMotor.move(left);
    rightMotor.move(right);
}

void turnToHeading(double targetAngle)
{
    //Creat P loop for turning, tolerance so that robot doesn't wobble trying to hit exact angle
    const float kP = 100;
    const float tolerance = 0.025;

    //Turn robot to reach target
    while(true)
    {
        odomUpdate(); //update position and heading
        double error = targetAngle - get_heading_rad(); //calculate error which is used to determine how fast the robot moves
        if(fabs(error) < tolerance) //stop if within tolerance
            break;
        
        double motorPower = kP * error; //motor moving speed equals the kP times how far from target angle is
        driveRobot(motorPower, -motorPower); //move robot
        pros::delay(10);
    }

    driveRobot(0,0);
}

//move to pose fuctions: moves to a point and heading
void moveToPose(float targetX, float targetY, float targetHeading)
{
    while(true)
    {
        //calculate distance to target
        double xDistance = targetX - pos_x;
        double yDistance = targetY - pos_y;
        double disToTarget = distance(pos_x, pos_y, targetX, targetY);

        //stop if within tolerance
        if(disToTarget < posTolerance) 
            break;
        
        //calculate angle lookahead to point
        double angleToTarget = atan2(yDistance, xDistance);

        //calculate angle to target (normalizes between -π and π)
        double headingError = angleToTarget - heading;
        while (headingError > M_PI) headingError -= 2 * M_PI;
        while (headingError < -M_PI) headingError += 2 * M_PI;

        //calculate movement speed
        double fowardSpeed = clamp(disToTarget * 10, -maxSpeed, maxSpeed);
        double turnSpeed = clamp(angleToTarget * kTurn * 100, -maxSpeed, maxSpeed);

        //calculate drive power
        double leftPower = fowardSpeed - turnSpeed;
        double rightPower = fowardSpeed + turnSpeed;

        //move motors, extra 120 mutliplier to account for turning values being small due to being in radians
        leftMotor.move(leftPower * 120);
        rightMotor.move(rightPower * 120);

        pros::delay(10);
    }

    //final heading corrections
    double finalError = targetHeading - heading;
    while (finalError > M_PI) finalError -= 2 * M_PI;
    while (finalError < -M_PI) finalError += 2 * M_PI;

    while (fabs(finalError) > 0.05) {
        double turnPower = clamp(finalError * kTurn * 100, -maxSpeed, maxSpeed);
        leftMotor.move_voltage(-turnPower / 100.0 * 12000);
        rightMotor.move_voltage(turnPower / 100.0 * 12000);

        pros::delay(10);
        finalError = targetHeading - heading;
    }

    leftMotor.move_voltage(0);
    rightMotor.move_voltage(0);
}

void moveToPoint(double targetX, double targetY) 
{
    while (true) 
    {
        double dx = targetX - pos_x;
        double dy = targetY - pos_y;
        double dist = sqrt(dx * dx + dy * dy);
        if (dist < posTolerance) break;

        double targetAngle = atan2(dy, dx);
        double angleError = targetAngle - heading;

        // Normalize angle to -pi to pi
        while (angleError > M_PI) angleError -= 2 * M_PI;
        while (angleError < -M_PI) angleError += 2 * M_PI;

        // Drive forward in the direction of the target
        double forwardPower = clamp(dist * kDrive, -maxSpeed, maxSpeed);
        double turnPower = clamp(angleError * kTurn * 100, -maxSpeed, maxSpeed);

        double leftPower = forwardPower - turnPower;
        double rightPower = forwardPower + turnPower;

        leftMotor.move_voltage(clamp(leftPower, -maxSpeed, maxSpeed));
        rightMotor.move_voltage(clamp(rightPower, -maxSpeed, maxSpeed));

        pros::delay(10);
    }

    // Stop motors
    leftMotor.move_voltage(0);
    rightMotor.move_voltage(0);
}

