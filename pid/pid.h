

#ifndef PID_H_
#define PID_H_

//Select your variable type..
typedef float ControlTypeDef;
//typedef int32_t controlType
//typedef q31 controlType

enum et_PIDSaturationStatus {
	PID_NotSaturated,
	PID_SaturatedMax,
	PID_SaturatedMin
};

struct s_PID {
	ControlTypeDef Reference;
	ControlTypeDef Measured;
	ControlTypeDef Output;

	ControlTypeDef Error;
	ControlTypeDef Error_Last;
	ControlTypeDef Integral;
	ControlTypeDef Derivative;

	ControlTypeDef Kp;
	ControlTypeDef Ki;
	ControlTypeDef Kd;
	ControlTypeDef Ts; 

	ControlTypeDef Saturation_Test;
	ControlTypeDef Actuator_Max;
	ControlTypeDef Actuator_Min;

	enum et_PIDSaturationStatus e_SaturatedStatus;
};

struct s_PIDOutput {
	ControlTypeDef Output;
	enum et_PIDSaturationStatus e_OutputSaturatedStatus;
};

void PID_Reset(struct s_PID *s);
void PID_Init(struct s_PID *s);
void PID_SetReference(struct s_PID *s, ControlTypeDef ref);
void PID_SetGains(struct s_PID *s, ControlTypeDef Kp, ControlTypeDef Ki, ControlTypeDef Kd, ControlTypeDef Ts);
void PID_SetActuatorLimits(struct s_PID *s, ControlTypeDef Min, ControlTypeDef Max);
struct s_PIDOutput PID_Calculate(struct s_PID *s);
struct s_PIDOutput PI_Calculate(struct s_PID *s);

#endif /* PID_H_ */
