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
const float wheelBase = 11;

bool rightDist = true;
bool frontDist = false;

double prevX;
double prevY;


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

bool robotIsMoving() 
{
    //make previous values
    static double lastX = 0.0;
    static double lastY = 0.0;
    static double last_heading = 0.0;
    static uint32_t last_time = pros::millis();

    //fetched change in position
    double dx = pos_x - lastX;
    double dy = pos_y - lastY;
    double d_heading = fabs(imu.get_rotation() - last_heading);

    uint32_t current_time = pros::millis();
    double dt = (current_time - last_time) / 1000.0;

    lastX = pos_x;
    lastY = pos_y;
    last_heading = imu.get_rotation();
    last_time = current_time;

    double linear_speed = sqrt(dx * dx + dy * dy) / dt;
    double angular_speed = d_heading / dt;

    // Thresholds (tune as needed)
    return (linear_speed > 1.0 || angular_speed > 1.0); // units: mm/s or deg/s
}


void odomInit() 
{
    //calibrate inertial sensor before running odom
  imu.reset();
  while (imu.is_calibrating()) pros::delay(20);
}

void odomUpdate() 
{
    if (!robotIsMoving()) 
        return;

    // et robot heading in radians from inertial sesnor
    heading = imu.get_heading() * (M_PI / 180.0);

    // Convert sensor readings to inches (from mm)
    double right_dist = dist_right.get() / 25.4;
    double back_dist = dist_back.get() / 25.4;

    // Get X pose
    prevX = pos_x;
    pos_x = 144.0 - (right_dist + x_offset * cos(heading));
    

    //get y choord
    prevY = pos_y;
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
void moveToPose(double targetX, double targetY, double targetHeading, double radius, double speed, bool clockwise = true) 
{
    // Get the vector from robot to target
    double dx = targetX - pos_x;
    double dy = targetY - pos_y;

    // Distance from robot to target
    double chordLength = sqrt(dx * dx + dy * dy);

    // Angle swept along the arc
    double angle_sweep = 2 * asin(chordLength / (2 * radius));

    // Normalize direction
    if (!clockwise) 
        angle_sweep = -angle_sweep;

    // Midpoint angle from robot to arc center
    double headingOffset = clockwise ? M_PI_2 : -M_PI_2;

    // Get arc center position
    double cx = pos_x + radius * cos(heading + headingOffset);
    double cy = pos_y + radius * sin(heading + headingOffset);

    // Starting angle from arc center to robot
    double startAngle = atan2(pos_y - cy, pos_x - cx);
    double currentAngle = startAngle;

    double targetAngle = startAngle + angle_sweep;

    // Keep turning along arc until the current angle reaches the target angle
    while (true) {
        // Update odometry
        double robotX = pos_x;
        double robotY = pos_y;

        currentAngle = atan2(robotY - cy, robotX - cx);

        // Check if the angle swept is complete
        double angleDelta = targetAngle - currentAngle;

        // Normalize
        while (angleDelta > M_PI) angleDelta -= 2 * M_PI;
        while (angleDelta < -M_PI) angleDelta += 2 * M_PI;

        if (fabs(angleDelta) < 0.05) break;

        // Turn rate is proportional to arc radius
        double leftSpeed = clockwise ? speed : speed * (radius - wheelBase / 2) / (radius + wheelBase / 2);
        double rightSpeed = clockwise ? speed * (radius + wheelBase / 2) / (radius - wheelBase / 2) : speed;

        // Clamp to max
        leftSpeed = std::fmax(std::fmin(leftSpeed, 127), -127);
        rightSpeed = std::fmax(std::fmin(rightSpeed, 127), -127);

        leftMotor.move(leftSpeed);
        rightMotor.move(rightSpeed);

        pros::delay(10);
    }

    // Face target heading at the end
    turnToHeading(targetHeading);

    // Stop
    leftMotor.move(0);
    rightMotor.move(0);

    return;
}


void oldMoveToPoint(double targetX, double targetY) 
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

void moveToPoint(double targetX, double targetY) 
{
    //PID constants
    const double kP_linear = 10;
    const double kD_linear = 0.5;
    const double kP_angular = 3.0;

    // Control loop settings (inches)
    const double distance_tolerance = 1.0; 
    const double max_linear_speed = 120;    // Tune to your robot
    const double max_angular_speed = 85;

    double prev_error = 0;

    while (true) 
    {
        // Calculate distance to target
        double dx = targetX - pos_x;
        double dy = targetY - pos_y;

        // Distance and angle to target
        double distance = sqrt(dx * dx + dy * dy);
        double targetAngle = atan2(dy, dx);
        double angle_error = targetAngle - heading;

        // Normalize angle to -π, π
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        // Stop condition
        if (distance < distance_tolerance)
        {
            break;
        }


        // Linear velocity with basic PD
        double error_derivative = (distance - prev_error) / (0.01);
        double linear_speed = kP_linear * distance + kD_linear * error_derivative;
        linear_speed = std::fmin(std::fmax(linear_speed, -max_linear_speed), max_linear_speed);


        // Angular velocity basic P
        double angular_speed = kP_angular * angle_error;
        angular_speed = std::fmin(std::fmax(angular_speed, -max_angular_speed), max_angular_speed);


        // Convert to left/right motor speeds (arcade-style drive)
        double left_power = linear_speed - angular_speed;
        double right_power = linear_speed + angular_speed;

        // Clamp speeds
        left_power = std::fmax(std::fmin(left_power, maxSpeed), -maxSpeed);
        right_power = std::fmax(std::fmin(right_power, maxSpeed), -maxSpeed);

        leftMotor.move(left_power);
        rightMotor.move(right_power);

        prev_error = distance;
        pros::delay(10);
    }

    // Stop motors
    leftMotor.move(0);
    rightMotor.move(0);
    return;
}

void curveToPose(float targetX, float targetY, float targetHeading,float arcDialation)
{
    // PID constants
    const double kP_linear = 50;
    const double kD_linear = 0.5;
    const double kP_angular = 3.0;

    // Control loop settings
    const double distance_tolerance = 1.0; // inches
    const int loop_delay = 10;

    double prev_error = 0;

    while(true)
    {
        // Calculate distance to target
        double dx = targetX - pos_x;
        double dy = targetY - pos_y;

        // Distance and angle to target
        double distance = sqrt(dx * dx + dy * dy);
        double targetAngle = atan2(dy, dx);
        double angle_error = targetAngle - heading;

        // Normalize angle to -π, π
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        // Stop condition
        if (distance < distance_tolerance)
            break;
        //
    }
}

