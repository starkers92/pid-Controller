
#include "pid.h"

void PID_Reset(struct s_PID *s) {
	s->Error=0;
	s->Error_Last=0;
	s->Integral=0;
}

void PID_Init(struct s_PID *s) {
	s->Reference=0;
	s->Measured=0;
	s->Output=0;

	s->Error=0;
	s->Error_Last=0;
	s->Integral=0;
	s->Derivative=0;

	s->Kp=0;
	s->Ki=0;
	s->Kd=0;
	s->Ts=0;

	s->Actuator_Max=0;
	s->Actuator_Min=0;

	s->e_SaturatedStatus = PID_NotSaturated;
}

void PID_SetReference(struct s_PID *s, ControlTypeDef ref) {
	s->Reference=ref;
}

void PID_SetGains(struct s_PID *s, ControlTypeDef Kp, ControlTypeDef Ki, ControlTypeDef Kd, ControlTypeDef Ts) {
	s->Kp=Kp;
	s->Ki=Ki;
	s->Kd=Kd;
	s->Ts=Ts;
}

void PID_SetActuatorLimits(struct s_PID *s, ControlTypeDef Min, ControlTypeDef Max) {
	s->Actuator_Max=Max;
	s->Actuator_Max=Min;
}

struct s_PIDOutput PID_Calculate(struct s_PID *s) {
	s->Error = s->Reference - s->Measured; //Error = Reference - Measured

	s->Integral = s->Integral + (s->Error)*(s->Ts); //Integral = Integral + Error*Ts

	s->Derivative = (s->Error - s->Error_Last)/(s->Ts); //Derivative = (Error-Error_Last)/Ts

	s->Error_Last = s->Error; //Error_Last = Error

	s->Saturation_Test = (s->Kp)*(s->Error) + (s->Ki)*(s->Integral) + (s->Kd)*(s->Derivative); 	//Saturation_Test = Kp * Error + Ki*Integral

	if( ( (s->Saturation_Test) <= s->Actuator_Min ) ||  ((s->Saturation_Test) >= s->Actuator_Max ) ) { //If Sat_Test < Min OR Sat_test > Max 
		//Anti-windup mechanism
		s->Integral = s->Integral - (s->Error) * (s->Ts); //Integral = Integral - Error*Ts;
		s->Output = s->Saturation_Test - (s->Ki)*(s->Integral);
	}
	else {
	//Actually do the PI
	//Output = Kp*error + Ki*integral
	s->Output = s->Saturation_Test;
	}
	//Check for actuator saturation
	if( (s->Output) <= (s->Actuator_Min) ) {
		(s->Output) = (s->Actuator_Min); //Apply saturation limit
		s->e_SaturatedStatus = PID_SaturatedMin; //Set Saturation Flag
	}
	else if ( (s->Output) >= (s->Actuator_Max) ) {
		(s->Output) = (s->Actuator_Max); //Apply saturation limit
		s->e_SaturatedStatus = PID_SaturatedMax; //Set Saturation Flag
	}
	else {
		s->e_SaturatedStatus = PID_NotSaturated; //Set Saturation Flag
	}
	
	struct s_PIDOutput s_Output = {
			s->Output,
			s->e_SaturatedStatus,
	};
	return s_Output;
}


struct s_PIDOutput PI_Calculate(struct s_PID *s) {
	s->Error = s->Reference - s->Measured; //Error = Reference - Measured

	s->Integral = s->Integral + (s->Error)*(s->Ts); //Integral = Integral + Error*Ts

	s->Error_Last = s->Error; //Error_Last = Error

	s->Saturation_Test = (s->Kp)*(s->Error) + (s->Ki)*(s->Integral); 	//Saturation_Test = Kp * Error + Ki*Integral

	if( ( (s->Saturation_Test) <= s->Actuator_Min ) ||  ((s->Saturation_Test) >= s->Actuator_Max ) ) { //If Sat_Test < Min OR Sat_test > Max
		//Anti-windup mechanism
		s->Integral = s->Integral - (s->Error) * (s->Ts); //Integral = Integral - Error*Ts;
		s->Output = s->Saturation_Test - (s->Ki)*(s->Integral);
	}

	//Actually do the PI
	//Output = Kp*error + Ki*integral
	s->Output = s->Saturation_Test;

	//Check for actuator saturation
	if( (s->Output) <= (s->Actuator_Min) ) {
		(s->Output) = (s->Actuator_Min); //Apply saturation limit
		s->e_SaturatedStatus = PID_SaturatedMin; //Set Saturation Flag
	}
	else if ( (s->Output) >= (s->Actuator_Max) ) {
		(s->Output) = (s->Actuator_Max); //Apply saturation limit
		s->e_SaturatedStatus = PID_SaturatedMax; //Set Saturation Flag
	}
	else {
		s->e_SaturatedStatus = PID_NotSaturated; //Set Saturation Flag
	}

	struct s_PIDOutput s_Output = {
			s->Output,
			s->e_SaturatedStatus,
	};
	return s_Output;
}
