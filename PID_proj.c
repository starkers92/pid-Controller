/*
 ============================================================================
 Name        : PID_proj.c
 Author      : Chris
 Version     :
 Copyright   : Free as in beer
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include "pid/pid.h"

#define myKp 1
#define myKi 1
#define myKd 1
#define myTs 1
#define myMin 1
#define myMax 1


int main(void) {

	/*
	 * Declare struct for PID controller to hold variables
	 * And another to hold the return values & status
	 */
	struct s_PID MyPIDController;
	struct s_PIDOutput MyPIDControllerOutput;
	/*
	 * -Initiate variables to zero
	 * -Set gains
	 * -Set actuator limits
	 */
	PID_Init(&MyPIDController);
	PID_SetGains(&MyPIDController, myKp, myKi, myKd, myTs);
	PID_SetActuatorLimits(&MyPIDController, myMin, myMax);
	/*
	 *
	 */
	while(1) {
		//Wait for timer flag
		MyPIDControllerOutput = PID_Calculate(&MyPIDController);
		// Check MyPIDControllerOutput.e_OutputSaturatedStatus for saturation
		// Send MyPIDControllerOutput.Output to to actuator function..
	}
	return EXIT_SUCCESS;
}
