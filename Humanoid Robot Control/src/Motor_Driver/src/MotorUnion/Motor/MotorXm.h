#pragma once
#include "Motor.h"

class MotorXm : public Motor
{
public:
	MotorXm();
	MotorXm(const unsigned char &MotorID, const string &MotorModel);
	~MotorXm(){};
};