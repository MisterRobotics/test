#include "Record_Replay.hpp"
#include "odom.hpp"
#include "odom.cpp"
#include "intake.hpp"
#include "pros/misc.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <fstream>
#include <vector>
#include <string>

bool piston1State;
bool piston2State;

//what's recorded during recording sessions
struct driveState
{
    int leftDrivePower;
    int rightDrivePower;
    int intakePower;
    bool piston1State;
    bool piston2State;
    double x;
    double y;
    double heading;
};

std::vector<driveState> driveLog;

void startRecording(const char* filename) 
{
    //clear previous log on Vex brain (prev log on SDCard)
    driveLog.clear();

    //set 15 second timer: auton time limit
    uint32_t start = pros::millis();
    while (pros::millis() - start < 15000) { // record for 15 seconds
        int leftPower = leftDrive.get_voltage();
        int rightPower = rightDrive.get_voltage();

        driveLog.push_back({leftPower, rightPower, intakeState, piston1State, piston2State,pos_x, pos_y, heading});

        pros::delay(15); 
    }

    FILE* file = fopen(filename, "w");
    if (file) {
        for (auto& state : driveLog) {
            fprintf(file, "%d,%d,%f,%f,%f\n",
                state.leftDrivePower, state.rightDrivePower,
                state.x, state.y, state.heading);
        }
        fclose(file);
        pros::lcd::print(0, "Recording saved!");
    } else {
        pros::lcd::print(0, "SD write failed!");
    }
}
