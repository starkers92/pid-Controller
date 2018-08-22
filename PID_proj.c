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
	PID_SetGains(&MyPIDController,100,10,1, 1);
	PID_SetActuatorLimits(&MyPIDController, 0, 50);
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
