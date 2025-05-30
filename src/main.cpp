#include "main.h"
#include "odom.hpp"
#include "autons.hpp"

//variables
int auton = 0;

pros::Controller master(pros::E_CONTROLLER_MASTER);







void initialize()
{
	pros::lcd::initialize();
	void odom_init();
	pros::Task coordPrintTask(coordinatePrint);

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
		odomUpdate();

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
		{
			turnToHeading(1.5708);
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
		{
			moveToPoint(124, 40);
			turnToHeading(1.5708);
		}
		pros::delay(10);
	}
}