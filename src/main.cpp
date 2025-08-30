#include "main.h"
#include "odom.hpp"
#include "autons.hpp"
#include <stdio.h>
#include <string.h>


//variables
int auton = 0;
int intakeState = 0;
bool piston1State = false;
bool piston2State = false;
bool yDrift;
bool xDrift;

pros::Motor intakeMid(8, pros::MotorGearset::blue);
pros::Motor intakeTop(9, pros::MotorGearset::green);

pros::adi::DigitalOut piston1('H', false);
pros::adi::DigitalOut piston2('H', false);

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftDrive({5,18,-7}, pros::MotorGearset::blue);
pros::MotorGroup rightDrive({-17,-12,13}, pros::MotorGearset::blue);

//Record + Replay auton functions and helpers

struct driveState
{
    int leftDrivePower;
    int rightDrivePower;
    int midIntakeVal;
	int upIntakeVal;
    bool piston1State;
    bool piston2State;
    double x;
    double y;
    double heading;
};

std::vector<driveState> driveLog;

void pistonControl()
{
	piston1.set_value(piston1State);
	piston2.set_value(piston2State);
}

void intakeControl()
{
    while(true)
    {
        //intake movement
        if(intakeState == 1)
        {
            //storage
            intakeMid.move(127);
			intakeTop.move(-18);
        }
        else if(intakeState == 2)
        {
            //middle goal score
            intakeMid.move(90);
            intakeTop.move(60);
        }
        else if(intakeState == 3)
        {
            //High score
            intakeMid.move(127);
            intakeTop.move(-127);
        }
        else if(intakeState == 4)
        {
            //outtake
            intakeMid.move(-127);
			intakeTop.move(127);
        }
        else if(intakeState == 0)
        {
            intakeMid.move(0);
            intakeTop.move(0);
        }
    }
} 

/*Helper function to move drivetrain in replay function
void moveDrive(float leftDrivePower, float rightDrivePower)
{
	leftDrive.move(leftDrivePower);
	rightDrive.move(rightDrivePower);
}*/

/*Helper function to move drivetrain via distance sensor and inertial sensor
void driveToPoint(float targetX, float targetY, float targetHeading)
{
	const float positionKP = 30;
	const float headingKP = 5;

	//distance to target using distance formula
	float dx = targetX - pos_x;
	float dy = targetY - pos_y;
	float lineDistance = sqrt(dx*dx + dy*dy);
	float angleToPoint = atan2(dy,dx);

	//calculate error for turning
	float angleError = angleToPoint - heading;
	float headingError = targetHeading - heading;

	//Normalize angles (in radian) between (-π, π)
	if (angleError > M_PI) angleError -= 2 * M_PI;
  	if (angleError < -M_PI) angleError += 2 * M_PI;
 	if (headingError > M_PI) headingError -= 2 * M_PI;
  	if (headingError < -M_PI) headingError += 2 * M_PI;

	//set movement power for turn and foward/backward
	float straightPower = lineDistance * positionKP;
	float turnPower = headingError * headingKP;

	//move robot
	moveDrive(straightPower + turnPower, straightPower - turnPower);
}*/

// --- RECORD ---
void record(const char* filename) 
{
    // clear previous memory log
    driveLog.clear();

    // open file in truncate mode -> clears previous file of the same name
    FILE* file = fopen(filename, "w");
    if (!file) {
        pros::lcd::print(4, "SD write failed!");
        return;
    }

    // 15 second recording window
    uint32_t start = pros::millis();
    const int interval = 15; // ms
    while (pros::millis() - start < 15000) 
	{
        // read values (use ints for powers so fprintf/fscanf stay consistent)
        int leftPower = leftDrive.get_voltage();   // or get_power() depending on your API
        int rightPower = rightDrive.get_voltage();
        int midIntVelo = intakeMid.get_voltage();
		int upIntVelo = intakeTop.get_voltage();
        bool p1 = piston1State;
        bool p2 = piston2State;
		double xCoord = pos_x;
		double yCoord = pos_y;
		double headingCoord = heading;

        driveLog.push_back
		({
			leftPower, 
			rightPower, 
			midIntVelo,
			upIntVelo, 
			p1, 
			p2, 
			xCoord, 
			yCoord, 
			headingCoord
		});

        // write line: left,right,intake,p1,p2,x,y,heading
        // use %d for ints, %f (or %.3f) for doubles
        fprintf(file, "%d,%d,%d,%d,%d,%.3f,%.3f,%.6f\n",
                leftPower, rightPower, midIntVelo, upIntVelo, p1 ? 1 : 0, p2 ? 1 : 0,
                pos_x, pos_y, heading);

        pros::delay(interval);
    }

    fclose(file);
    pros::lcd::print(4, "Recording saved!");
}

// --- REPLAY ---
void replay(const char* filename) 
{
    FILE* file = fopen(filename, "r");
    if (!file) {
        pros::lcd::print(0, "SD read failed!");
        return;
    }

    // temporaries for scanning
    int leftPower, rightPower;
    int midIntVelo;
	int upIntVelo;
    int p1Int, p2Int;
    double x, y, theta;

    const int interval = 15; // match recording interval

    while (fscanf(file, "%d,%d,%d,%d,%d,%lf,%lf,%lf\n",
                  &leftPower, &rightPower, &midIntVelo, &upIntVelo, &p1Int, &p2Int,
                  &x, &y, &theta) == 8) 
	{

        // set drive — use same unit as recorded
        leftDrive.move_voltage(leftPower);
        rightDrive.move_voltage(rightPower);

        // set intake based on the recorded intakeVal (example mapping)
        intakeMid.move(midIntVelo);
		intakeTop.move(upIntVelo);
		pros::Task setIntakeSpeed([]
			{
				intakeControl();
			});

        // convert ints back to bools and apply pistons
        piston1State = (p1Int != 0);
        piston2State = (p2Int != 0);
        pistonControl(); 

        // optional: update odom display or log playback position variables
        // pos_x = x; pos_y = y; heading = theta;  // only if you want to force odom (not recommended)

        pros::delay(interval);
    }

    fclose(file);

    // stop everything after replay
    leftDrive.move_voltage(0);
    rightDrive.move_voltage(0);
    intakeTop.move_voltage(0);
    intakeMid.move_voltage(0);

    pros::lcd::print(0, "Replay finished!");
}




void driverIntake()
{
    //intake controls
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			intakeState = 1;
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			intakeState = 3;
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			intakeState = 2;
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			intakeState = 4;
		}
		else
		{
			intakeState = 0;
		}


		pros::lcd::print(4, "intake state:%d", intakeState);
}



void initialize()
{
	pros::lcd::initialize();
	void odom_init();
	pros::Task coordPrintTask(coordinatePrint);
	pros::Task updatePos([]
	{
		while(true)
		{
			odomUpdate();
			pros::delay(10);
		}
	});
	pros::Task runIntake([]
	{
		while(true)
		{
			intakeControl();
			pros::delay(10);
		}
	});

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}


 
void autonomous() 
{
	switch (auton)
	{
	case 0:
		auton0();
		break;
	case 1:
		auton1();
		break;
	case 2:
		auton2();
		break;
	case 3:
		auton3();
		break;
	default:
		break;
	}
}


void opcontrol() 
{
	while(true)
	{
		//variables
		int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        //Drive Code for single stick
        leftDrive.move(rightY + rightX);
        rightDrive.move(rightY - rightX);


		driverIntake();

		/*if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
		{
			turnToHeading(1.5708);
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
		{
			moveOneAxis(54);
			//moveToPoint(128, 44);
			master.rumble(". - . -");
		}*/

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) 
		{
    		pros::Task recordAuton([] 
			{
        		record("auton1");
    		});
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
		{
			replay("auton1");
		}


		
		
		//delay to not overload CPU
		pros::delay(10);
	}
}