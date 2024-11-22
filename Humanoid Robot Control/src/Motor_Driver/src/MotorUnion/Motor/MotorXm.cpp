#include "MotorXm.h"

MotorXm::MotorXm()
	: Motor(57600, 0, 64, 102, 104, 108, 112, 116, 126, 128, 132, 11, 1, 2, 4, 4, 4, 4, 2, 4, 4) {}

MotorXm::MotorXm(const unsigned char &MotorID, const string &MotorModel)
	: Motor(4000000, MotorID, 64, 102, 104, 108, 112, 116, 126, 128, 132, 11, 1, 2, 4, 4, 4, 4, 2, 4, 4)
{
	if (MotorModel == "Xm540" || MotorModel == "Xm430")
	{
		Motor_CenterScale = 2048;
		Max_Position_Limit = 4095;
		Min_Position_Limit = 0;
		Max_Velocity_Limit = 1023;
		Min_Velocity_Limit = 0;
		Max_Accel_Limit = 32767;
		Max_Current_Limit = 2047;
		Min_Current_Limit = 0;
		Angle2MotorScale = (Max_Position_Limit - Min_Position_Limit) / 360.0;
		MotorScale2Angle = 1.0 / Angle2MotorScale;
		Scale2RPM = 0.229;
		Scale2RPMM = 214.577;
	}
}