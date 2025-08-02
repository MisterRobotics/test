#include "main.h"
#include "odom.hpp"
#include "autons.hpp"

int intakeState = 0;

pros::Motor intakeMid(8, pros::MotorGearset::blue);
pros::Motor intakeTop(9, pros::MotorGearset::green);

void intakeControl()
{
    while(true)
    {
        //intake movement
        if(intakeState == 1)
        {
            //storage
            intakeMid.move(127);
        }
        else if(intakeState == 2)
        {
            //middle goal score
            intakeMid.move(127);
            intakeTop.move(127);
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
        }
        else if(intakeState == 0)
        {
            intakeMid.move(0);
            intakeTop.move(0);
        }
    }
}        
