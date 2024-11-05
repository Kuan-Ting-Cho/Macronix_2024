#include "MotorPro.h"
MotorPro::MotorPro()
	: Motor(3000000, 7, 512, 550, 552, 556, 560, 564, 574, 576, 580, 11, 1, 2, 4, 4, 4, 4, 2, 4, 4) {}

MotorPro::MotorPro(const unsigned char &MotorID, const string &MotorModel)
	: Motor(3000000, MotorID, 512, 550, 552, 556, 560, 564, 574, 576, 580, 11, 1, 2, 4, 4, 4, 4, 2, 4, 4)
{
	if (MotorModel == "Pro200")
	{
	}
	else if (MotorModel == "Pro100")
	{
		Motor_CenterScale = 0;
		Max_Position_Limit = 501433;
		Min_Position_Limit = -501433;
		Max_Velocity_Limit = 2920;
		Min_Velocity_Limit = -2920;
		Max_Extend_Limit = 2147483647;
		Min_Extend_Limit = -2147483648;
		Max_Value_In_1_rev = 501433;  // Maximum value in one round in extended mode
		Min_Value_In_1_rev = -501433; // Minimum value in one round in extended mode
		Max_Accel_Limit = 4255632;
		Max_Current_Limit = 15900;
		Min_Current_Limit = -15900;

		Angle2MotorScale = (Max_Position_Limit - Min_Position_Limit) / 360.0;
		MotorScale2Angle = 1.0 / Angle2MotorScale;
		Rev_Resol_Extended = Max_Value_In_1_rev - Min_Value_In_1_rev;
		Scale2RPM = 0.01;
		Scale2RPMM = 1;
	}
	else if (MotorModel == "Pro20")
	{
		Motor_CenterScale = 0;
		Max_Position_Limit = 303454;
		Min_Position_Limit = -303454;
		Max_Velocity_Limit = 2920;
		Min_Velocity_Limit = -2920;
		Max_Extend_Limit = 2147483647;
		Min_Extend_Limit = -2147483648;
		Max_Accel_Limit = 4306173;
		Max_Current_Limit = 4500;
		Min_Current_Limit = -4500;
		Max_Value_In_1_rev = 303454;  // Maximum value in one round in extended mode
		Min_Value_In_1_rev = -303454; // Minimum value in one round in extended mode

		Angle2MotorScale = (Max_Position_Limit - Min_Position_Limit) / 360.0;
		MotorScale2Angle = 1.0 / Angle2MotorScale;
		Rev_Resol_Extended = Max_Value_In_1_rev - Min_Value_In_1_rev;
		Scale2RPM = 0.01;
		Scale2RPMM = 1;
	}
}