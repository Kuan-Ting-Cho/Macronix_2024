#pragma once
#include "dynamixel/DynamixelSDK.h"
#include "./motor/motor.h"
#include <iostream>

using namespace std;

class Motor : public motor
{
public:
	Motor();
	/*
	@ Baudrate,
	@ Id,
	@ Addr_torque_enable,
	@ Addr_goal_current,
	@ Addr_goal_velocity,
	@ Addr_profile_accel,
	@ Addr_profile_velocity,
	@ Addr_goal_position,
	@ Addr_present_current,
	@ Addr_present_velocity,
	@ Addr_present_position,
	@ Addr_operating_mode,
	@ Len_torque_enable,
	@ Len_goal_current,
	@ Len_goal_velocity,
	@ Len_profile_accel,
	@ Len_profile_velocity,
	@ Len_goal_position,
	@ Len_present_current,
	@ Len_present_velocity,
	@ Len_present_position
	*/
	Motor(
		const unsigned int &baudrate,
		const unsigned int &id,
		const uint16_t &addr_torque_enable,
		const uint16_t &addr_goal_current, // new
		const uint16_t &addr_goal_velocity,
		const uint16_t &addr_profile_accel,
		const uint16_t &addr_profile_velocity,
		const uint16_t &addr_goal_position,
		const uint16_t &addr_present_current,
		const uint16_t &addr_present_velocity,
		const uint16_t &addr_present_position,
		const uint16_t &addr_operating_mode,
		const uint16_t &len_torque_enable,
		const uint16_t &len_goal_current, // new
		const uint16_t &len_goal_velocity,
		const uint16_t &len_profile_accel,
		const uint16_t &len_profile_velocity,
		const uint16_t &len_goal_position,
		const uint16_t &len_present_current,
		const uint16_t &len_present_velocity,
		const uint16_t &len_present_position);
	~Motor(){};
	//--------------------------//
	const int &GetMotorID() const;
	const bool &GetMotorConnected() const;

	/* Following functions "doesn't" need to be implemented */
	void ConnectDynamixel(
		dynamixel::PortHandler *portHandler,
		dynamixel::PacketHandler *packetHandler,
		dynamixel::GroupBulkRead *groupBulkRead,
		dynamixel::GroupBulkWrite *groupBulkWrite);
	void ConnectDynamixel();
	bool WriteData();
	void AddParam();
	void ReadData();
	// void WriteMode(uint8_t mode);
	void WriteMode();

private:
	void AddParamPresentAngle();
	void AddParamPresentVelocity();
	void AddParamPresentCurrent();
	void ReadPresentAngle();
	void ReadPresentVelocity();
	void ReadPresentCurrent();
	void WriteScale();
	void WriteVelocity();
	void WriteAccel();
	void WriteCurrent(); // new
	void WriteTorqueEnable();

protected:
	dynamixel::PortHandler *portHandler;
	dynamixel::PacketHandler *packetHandler;
	dynamixel::GroupBulkRead *groupBulkRead;
	dynamixel::GroupBulkWrite *groupBulkWrite;

	/* Dynamixel attributes (see e-manual) */
	const int BAUDRATE;
	const int Motor_ID;
	const uint16_t ADDR_TORQUE_ENABLE;
	const uint16_t ADDR_GOAL_CURRENT; // new
	const uint16_t ADDR_GOAL_VELOCITY;
	const uint16_t ADDR_PROFILE_ACCEL;
	const uint16_t ADDR_PROFILE_VELOCITY;
	const uint16_t ADDR_GOAL_POSITION;
	const uint16_t ADDR_PRESENT_CURRENT; // equal to current address
	const uint16_t ADDR_PRESENT_VELOCITY;
	const uint16_t ADDR_PRESENT_POSITION;
	const uint16_t ADDR_OPERATING_MODE;
	const uint16_t LEN_TORQUE_ENABLE;
	const uint16_t LEN_GOAL_CURRENT; // new
	const uint16_t LEN_GOAL_VELOCITY;
	const uint16_t LEN_PROFILE_ACCEL;
	const uint16_t LEN_PROFILE_VELOCITY;
	const uint16_t LEN_GOAL_POSITION;
	const uint16_t LEN_PRESENT_CURRENT;
	const uint16_t LEN_PRESENT_VELOCITY;
	const uint16_t LEN_PRESENT_POSITION;

	bool connected;
	char read_count;
};