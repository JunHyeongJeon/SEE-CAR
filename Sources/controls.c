/*
 * controls.c
 *
 *  Created on: Jun 30, 2015
 *      Author: luaviskang
 */

#include "controls.h"

void control_speed(Wheel * left, Wheel * right) {
	
}

long int pid_control(long int speedRf,long int feedback){
	
	static long int preError[10]={0,0,0,0,0,0,0,0,0};
	static long int errorSum=0;
	volatile i;
	//pid controller	
	long int kp=300;
	long int kd=2;
	long int ki=8;
	long int pid;
	
	long int error;
	long int errorDif;		
	
	error = speedRf - feedback; 
	errorDif = error - preError[0];
	
	errorSum = 0;
	for(i=0;i<9;i++)
		errorSum +=preError[i];
	
	errorSum +=error; 

	pid = error * kp;
	pid+= errorDif * kd;
	pid+= errorSum * ki;
	
	//shift
	for(i=9;i>0;i--)
		preError[i] = preError[i-1];	
	//save data	
	preError[0] = error;

	return pid;
}

