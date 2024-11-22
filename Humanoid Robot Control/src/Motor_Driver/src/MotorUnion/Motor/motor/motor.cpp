#include "motor.h"
#include <iostream>
using namespace std;
motor::motor()
{
	Angle2MotorScale = 0.0f;
	MotorScale2Angle = 0.0f;
	Scale2RPM = 0.0f;
	Scale2RPMM = 0.0f;

	Motor_Operating_Mode = 3;
	Motor_CenterScale = 0;
	Motor_Angle = 0.0f;
	Motor_Scale = 0;
	Motor_Velocity = 0.0f;
	Motor_Current = 0.0f;
	Motor_Accel = 0.0f;
	Motor_Present_Angle = 0.0f;
	Motor_Present_Velocity = 0.0f;
	Motor_Present_Current = 0.0f;
	Motor_TorqueEnable = false;

	Max_Position_Limit = 0;
	Min_Position_Limit = 0;
	Max_Velocity_Limit = 0;
	Min_Velocity_Limit = 0;
	Max_Extend_Limit = 0;
	Min_Extend_Limit = 0;
	Max_Accel_Limit = 0;
	Max_Current_Limit = 0;
	Min_Current_Limit = 0;

	is_Arrival = true;
	is_Write_Scale = false;
	is_Write_Velocity = false;
	is_Write_Accel = false;
	is_Write_Current = false;
	is_Write_TorqueEnable = false;
}
//-----------------------------------------------------//
const int &motor::GetMotor_Operating_Mode() const { return Motor_Operating_Mode; }

const float &motor::GetMotor_Scale2RPM() const { return Scale2RPM; }

const float &motor::GetMotor_Scale2RPMM() const { return Scale2RPMM; }

const short &motor::GetMotor_CenterScale() const { return Motor_CenterScale; }

const float &motor::GetMotor_Angle() const { return Motor_Angle; }

const float &motor::GetMotor_Velocity() const { return Motor_Velocity; }

const float &motor::GetMotor_Accel() const { return Motor_Accel; }

const float &motor::GetMotor_Current() const { return Motor_Current; }

const bool &motor::GetMotor_TorqueEnable() const { return Motor_TorqueEnable; }

const float &motor::GetMotor_PresentAngle() const { return Motor_Present_Angle; }

const float &motor::GetMotor_PresentVelocity() const { return Motor_Present_Velocity; }

const float &motor::GetMotor_PresentCurrent() const { return Motor_Present_Current; }

const bool &motor::GetMotor_Arrival() const { return is_Arrival; }
//-----------------------------------------------------//
void motor::SetMotor_Operating_Mode(const int &operating_mode)
{
	Motor_Operating_Mode = operating_mode;
}

void motor::SetMotor_CenterScale(const short &centerscale)
{
	Motor_CenterScale = centerscale;
}
void motor::SetMotor_Angle(const float &angle)
{
	switch (Motor_Operating_Mode)
	{
	case 1: // Velocity control mode
	case 3: // Position control mode
		Motor_Scale = round(angle * Angle2MotorScale + Motor_CenterScale);
		if (Motor_Scale >= Max_Position_Limit)
			Motor_Scale = Max_Position_Limit;
		else if (Motor_Scale <= Min_Position_Limit)
			Motor_Scale = Min_Position_Limit;
		else
			;
		Motor_Angle = (Motor_Scale - Motor_CenterScale) * MotorScale2Angle;
		break;
	case 4: // Extended position control mode
		Motor_Scale = round(angle / 360 * Rev_Resol_Extended);
		if (Motor_Scale >= Max_Extend_Limit)
			Motor_Scale = Max_Extend_Limit;
		else if (Motor_Scale <= Min_Extend_Limit)
			Motor_Scale = Min_Extend_Limit;
		else
			;
		Motor_Angle = (float)Motor_Scale / Rev_Resol_Extended * 360;

		break;
	}

	is_Arrival = false;
	is_Write_Scale = true;
}

void motor::SetMotor_Velocity(const float &velocity)
{
	switch (Motor_Operating_Mode)
	{
	case 1: // Velocity control mode
		if (velocity >= Max_Velocity_Limit)
			Motor_Velocity = Max_Velocity_Limit;
		else if (velocity <= Min_Velocity_Limit)
			Motor_Velocity = Min_Velocity_Limit;
		else
			Motor_Velocity = velocity;
		break;

	case 3: // Position control mode
	case 4: // Extended position control mode
		if (std::abs(velocity) >= Max_Velocity_Limit)
			Motor_Velocity = Max_Velocity_Limit;
		else
			Motor_Velocity = std::abs(velocity);
		break;
	}
	is_Write_Velocity = true;
}

void motor::SetMotor_Accel(const float &accel)
{
	if (accel >= Max_Accel_Limit)
		Motor_Accel = Max_Accel_Limit;
	else
		Motor_Accel = accel;

	is_Write_Accel = true;
}
void motor::SetMotor_Current(const float &current)
{
	// cout << Motor_Operating_Mode << endl;
	switch (Motor_Operating_Mode)
	{
	case 0: // Current control mode
		if (current >= Max_Current_Limit)
			Motor_Current = Max_Current_Limit;
		else if (current <= Min_Current_Limit)
			Motor_Current = Min_Current_Limit;
		else
			Motor_Current = current;
		break;
	case 1: // Velocity control mode
	case 3: // Position control mode
	case 4: // Extended position control mode
		;
	}
	is_Write_Current = true;
}
void motor::SetMotor_TorqueEnable(const bool &enable)
{
	Motor_TorqueEnable = enable;
	is_Write_TorqueEnable = true;
}
