/*
 * hamed.c
 *
 * Created: 7/18/2016 4:06:16 PM
 *  Author: hameds23
 */ 
#include <hamed.h>
#include <functions.h>

void robot_speed(int a)
{
	int speed=100;
	if(Robot.W1.full<a)
	{
		Robot.W1_sp.full++;
	}
	if(Robot.W1.full>a)
	{
		Robot.W1_sp.full--;
	}
	speed=Robot.W1_sp.full;
}
	