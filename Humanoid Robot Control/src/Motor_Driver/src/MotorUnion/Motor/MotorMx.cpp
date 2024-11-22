#include "MotorMx.h"

MotorMx::MotorMx()
	: Motor(57600, 7, 64, 100, 104, 108, 112, 116, 11, 126, 128, 132, 1, 2, 4, 4, 4, 4, 2, 4, 4) {}

/* Dynamixel attributes (see e-manual)
const int BAUDRATE;
const unsigned char Motor_ID;
const uint16_t ADDR_TORQUE_ENABLE;
const uint16_t ADDR_GOAL_Current; //goal pwm replace
const uint16_t ADDR_GOAL_VELOCITY;
const uint16_t ADDR_PROFILE_ACCEL;
const uint16_t ADDR_PROFILE_VELOCITY;
const uint16_t ADDR_GOAL_POSITION;
const uint16_t ADDR_OPERATING_MODE;
const uint16_t ADDR_PRESENT_TORQUE; //present load replace
const uint16_t ADDR_PRESENT_VELOCITY;
const uint16_t ADDR_PRESENT_POSITION;

const uint16_t LEN_TORQUE_ENABLE;
const uint16_t LEN_GOAL_VELOCITY;
const uint16_t LEN_PROFILE_ACCEL;
const uint16_t LEN_PROFILE_VELOCITY;
const uint16_t LEN_GOAL_POSITION;
const uint16_t LEN_PRESENT_TORQUE;
const uint16_t LEN_PRESENT_VELOCITY;
const uint16_t LEN_PRESENT_POSITION;
*/

MotorMx::MotorMx(const unsigned char &MotorID, const string &MotorModel)
	: Motor(57600, MotorID, 64, 100, 104, 108, 112, 116, 11, 126, 128, 132, 1, 2, 4, 4, 4, 4, 2, 4, 4)
{
	if (MotorModel == "Mx28" || MotorModel == "Mx106")
	{
		Motor_CenterScale = 2048;
		Max_Position_Limit = 4095;
		Min_Position_Limit = 0;
		// Max_Velocity_Limit = 210;
		// Min_Velocity_Limit = 0;

		Max_Velocity_Limit = 1023;
		Min_Velocity_Limit = -1023;
		Max_Accel_Limit = 32767;
		// Min_Torque_Limit = -1000;
		Max_Current_Limit = 1000;

		Angle2MotorScale = (Max_Position_Limit - Min_Position_Limit) / 360.0;
		MotorScale2Angle = 1.0 / Angle2MotorScale;
		Scale2RPM = 0.229;
		Scale2RPMM = 214.577;
	}
}
