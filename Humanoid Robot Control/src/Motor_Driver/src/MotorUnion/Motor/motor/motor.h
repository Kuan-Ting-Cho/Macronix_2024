#pragma once
#include <cmath>

#define Rad2Angle (180.0 / M_PI)
#define Angle2Rad (M_PI / 180.0)

class motor
{
public:
	motor();
	~motor(){};
	//-----------------------------------------------------//
	const int &GetMotor_Operating_Mode() const;
	const float &GetMotor_Scale2RPM() const;
	const float &GetMotor_Scale2RPMM() const;
	const short &GetMotor_CenterScale() const;
	const float &GetMotor_Angle() const;
	const float &GetMotor_Velocity() const;
	const float &GetMotor_Accel() const;
	const float &GetMotor_Current() const;
	const bool &GetMotor_TorqueEnable() const;
	const float &GetMotor_PresentAngle() const;
	const float &GetMotor_PresentVelocity() const;
	const float &GetMotor_PresentCurrent() const;
	const bool &GetMotor_Arrival() const;
	//-----------------------------------------------------//
	void SetMotor_Operating_Mode(const int &);
	void SetMotor_CenterScale(const short &);
	void SetMotor_Angle(const float &);
	void SetMotor_Velocity(const float &);
	void SetMotor_Current(const float &);
	void SetMotor_Accel(const float &);
	void SetMotor_TorqueEnable(const bool &);
	//------------------------------------------------------------------------------------------------------------------//

protected:
	/* Conversion of units */
	float Angle2MotorScale;
	float MotorScale2Angle;
	int Rev_Resol_Extended; // Resolution for one rev in extended mode
	float Scale2RPM;
	float Scale2RPMM;

	/* Motor basic attributes */
	int Motor_Operating_Mode;
	short Motor_CenterScale;
	float Motor_Angle;			  // (degree)
	int Motor_Scale;			  // Goal Position (Motor Scale)
	float Motor_Velocity;		  // Goal Velocity
	float Motor_Accel;			  // Goal Acceleration
	float Motor_Current;		  // Goal Current
	float Motor_Present_Angle;	  // Present Position (degree)
	float Motor_Present_Velocity; // Present Velocity (rpm)
	float Motor_Present_Current;  // Percentage loading (%) (Present_Current / MaxCurrent)
	bool Motor_TorqueEnable;

	/* Max Min Limit*/
	int Max_Position_Limit; // Maximum value of position control
	int Min_Position_Limit; // Minimum value of position control
	int Max_Velocity_Limit; // Maximum value of velocity control
	int Min_Velocity_Limit; // Minimum value of velocity control
	int Max_Extend_Limit;	// Maximum value of extended mode control
	int Min_Extend_Limit;	// Minimum value of extended mode control
	int Max_Value_In_1_rev; // Maximum value in one round in extended mode
	int Min_Value_In_1_rev; // Minimum value in one round in extended mode
	int Max_Accel_Limit;
	int Max_Current_Limit;
	int Min_Current_Limit;

	/* Background flags */
	bool is_Arrival;
	bool is_Write_Scale;
	bool is_Write_Velocity;
	bool is_Write_Accel;
	bool is_Write_Current;
	bool is_Write_TorqueEnable;
};
