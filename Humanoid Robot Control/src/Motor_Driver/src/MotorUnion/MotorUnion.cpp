#include "MotorUnion.h"
#include <vector>
#include <unistd.h>
#include <termio.h>
using namespace std;
vector<unsigned char> MotorUnion::allport = {0, 1, 2, 3, 4, 5, 6};

/**
 * Motor constructor
 *
 * @param IDArray -
 * @param MotorModelArray - Array defined the type of motor using
 */
MotorUnion::MotorUnion(const vector<unsigned char> &IDArray,
					   const vector<string> &MotorModelArray)
	: _is_deleted_thread_BG(true),
	  waiting_frequency(10)
{
	for (unsigned char i = 0; i < IDArray.size(); i++)
	{
		if (MotorModelArray.at(i) == "Pro100" || MotorModelArray.at(i) == "Pro20")
			Motor_Union.push_back(new MotorPro(IDArray.at(i), MotorModelArray.at(i)));
	}

	if (ConnectAllMotors(MotorUnion::allport))
		BGON();
}

MotorUnion::~MotorUnion()
{
	if (!_is_deleted_thread_BG)
	{
		_is_deleted_thread_BG = true;
		thread_BG->join();
		delete thread_BG;
	}

	deleteInVector(Motor_Union);
	deleteInVector(portHandler);
	deleteInVector(groupBulkRead);
	deleteInVector(groupBulkWrite);
}

template <class T>
void MotorUnion::deleteInVector(vector<T *> tmp_vector)
{
	while (!tmp_vector.empty())
	{
		delete tmp_vector.back();
		tmp_vector.pop_back();
	}
}

////////////////////////////////////////////////////////////////////////////////
///  All Motors   //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
const bool MotorUnion::ConnectAllMotors(vector<unsigned char> &AllPortNumber)
{
	vector<unsigned char>::iterator it;
	for (it = AllPortNumber.begin(); it != AllPortNumber.end();)
	{
		// Set the port path
		string port_path = string("/dev/ttyUSB" + to_string(*it));
		// Initialize PortHandler & PacketHandler instance
		dynamixel::PortHandler *tmp_portHandler = dynamixel::PortHandler::getPortHandler(port_path.c_str());
		packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);
		dynamixel::GroupBulkRead *tmp_groupBulkRead = new dynamixel::GroupBulkRead(tmp_portHandler, packetHandler);
		dynamixel::GroupBulkWrite *tmp_groupBulkWrite = new dynamixel::GroupBulkWrite(tmp_portHandler, packetHandler);

		bool port_gate = false;
		for (int i = 0; i < Motor_Union.size(); i++)
		{
			if (Motor_Union.at(i)->GetMotorConnected() == false)
			{
				Motor_Union.at(i)->ConnectDynamixel(tmp_portHandler, packetHandler, tmp_groupBulkRead, tmp_groupBulkWrite);
				port_gate |= Motor_Union.at(i)->GetMotorConnected();
			}
		}
		if (port_gate == true)
		{
			it = AllPortNumber.erase(it);
			portHandler.push_back(tmp_portHandler);
			groupBulkRead.push_back(tmp_groupBulkRead);
			groupBulkWrite.push_back(tmp_groupBulkWrite);
		}
		else
		{
			++it;
			delete tmp_groupBulkWrite;
			delete tmp_groupBulkRead;
			delete tmp_portHandler;
		}
	}

	// Check every motor is connected
	bool connected = true;
	for (int i = 0; i < Motor_Union.size(); i++)
		connected &= Motor_Union.at(i)->GetMotorConnected();

	return connected;
}

const bool MotorUnion::CheckAllMotorsConnected() const
{
	bool connected = true;
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		Motor_Union.at(i)->ConnectDynamixel();
		connected &= Motor_Union.at(i)->GetMotorConnected();
	}
	return connected;
}

const bool MotorUnion::CheckAllMotorsArrival() const
{
	bool arrival = true;
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		arrival &= Motor_Union.at(i)->GetMotor_Arrival();
	}
	return arrival;
}

/**
 * Wait for i-th motor to arrive
 *
 * @param i - ID of the motor
 */
void MotorUnion::WaitMotorArrival(int i) const
{
	while (!Motor_Union.at(i)->GetMotor_Arrival())
	{
		if (GetAllMotorsTorqueEnable() == false)
			break;
		this_thread::sleep_for(chrono::milliseconds(waiting_frequency));
	}
}

void MotorUnion::WaitAllMotorsArrival() const
{
	while (!CheckAllMotorsArrival())
	{
		if (GetAllMotorsTorqueEnable() == false)
			break;
		this_thread::sleep_for(chrono::milliseconds(waiting_frequency));
	}
}

void MotorUnion::WaitAllMotorsArrival(const int &total_waiting_time_ms) const
{
	for (int i = 0; i < total_waiting_time_ms / waiting_frequency; i++)
	{
		if (GetAllMotorsTorqueEnable() == false)
			break;
		this_thread::sleep_for(chrono::milliseconds(waiting_frequency));
	}
}

//------------------------------------------------------------------------------//
/*
	Get All Motors Data
*/
const bool MotorUnion::GetAllMotorsTorqueEnable() const
{
	bool tmp = true;
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		tmp &= GetMotor_TorqueEnable(i);
	}
	return tmp;
}

//------------------------------------------------------------------------------//
/*
	Set Motors Data
*/
void MotorUnion::SetAllMotorsOperatingMode(const int &mode) const
{
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		SetMotor_Operating_Mode(i, mode);
	}
}

void MotorUnion::SetAllMotorsAngle(const float &angle) const
{
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		SetMotor_Angle(i, angle);
	}
}

void MotorUnion::SetAllMotorsVelocity(const float &velocity) const
{
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		SetMotor_Velocity(i, velocity);
	}
}

void MotorUnion::SetAllMotorsAccel(const float &accel) const
{
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		SetMotor_Accel(i, accel);
	}
}

void MotorUnion::SetAllMotorsTorqueEnable(const bool &enable) const
{
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		SetMotor_TorqueEnable(i, enable);
	}
	if (enable == true)
	{
		cout << "SetAllMotorsTorqueEnable: On" << endl;
	}
	else
		cout << "SetAllMotorsTorqueEnable: Off" << endl;
}

////////////////////////////////////////////////////////////////////////////////
///   Motor   //////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------//
/*
	Get Motor Data
*/
const int &MotorUnion::GetMotor_ID(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotorID();
}

const bool &MotorUnion::GetMotor_Connected(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotorConnected();
}

const float &MotorUnion::GetMotor_Scale2RPM(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Scale2RPM();
}

const float &MotorUnion::GetMotor_Scale2RPMM(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Scale2RPMM();
}

const short &MotorUnion::GetMotor_CenterScale(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_CenterScale();
}

const float &MotorUnion::GetMotor_Angle(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Angle();
}

const float &MotorUnion::GetMotor_Velocity(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Velocity();
}

const float &MotorUnion::GetMotor_Accel(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Accel();
}
const float &MotorUnion::GetMotor_Current(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Current();
}
const bool &MotorUnion::GetMotor_TorqueEnable(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_TorqueEnable();
}

const float &MotorUnion::GetMotor_PresentAngle(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_PresentAngle();
}

const float &MotorUnion::GetMotor_PresentVelocity(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_PresentVelocity();
}

const float &MotorUnion::GetMotor_PresentCurrent(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_PresentCurrent();
}
const int &MotorUnion::GetMotor_Operating_Mode(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Operating_Mode();
}
//------------------------------------------------------------------------------//
/*
	Set Motor Data
*/
void MotorUnion::SetMotor_Operating_Mode(const unsigned char &idx, int mode) const // can't set mode online
{
	// Motor_Union.at(idx)->SetMotor_TorqueEnable(false);
	// this_thread::sleep_for(chrono::milliseconds(50));
	Motor_Union.at(idx)->SetMotor_Operating_Mode(mode);
	// Motor_Union.at(idx)->WriteMode(mode);
	// this_thread::sleep_for(chrono::milliseconds(50));
	// Motor_Union.at(idx)->SetMotor_TorqueEnable(true);
}

void MotorUnion::SetMotor_CenterScale(const unsigned char &idx, const short &motor_center_scale) const
{
	Motor_Union.at(idx)->SetMotor_CenterScale(motor_center_scale);
}

void MotorUnion::SetMotor_Angle(const unsigned char &idx, const float &angle) const
{
	Motor_Union.at(idx)->SetMotor_Angle(angle);
}

void MotorUnion::SetMotor_Velocity(const unsigned char &idx, const float &velocity) const
{
	Motor_Union.at(idx)->SetMotor_Velocity(velocity);
}

void MotorUnion::SetMotor_Accel(const unsigned char &idx, const float &accel) const
{
	Motor_Union.at(idx)->SetMotor_Accel(accel);
}
void MotorUnion::SetMotor_Current(const unsigned char &idx, const float &current) const
{
	Motor_Union.at(idx)->SetMotor_Current(current);
}
void MotorUnion::SetMotor_TorqueEnable(const unsigned char &idx, const bool &enable) const
{
	Motor_Union.at(idx)->SetMotor_TorqueEnable(enable);
}

////////////////////////////////////////////////////////////////////////////////
///   Background   /////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void MotorUnion::BGON()
{
	_is_deleted_thread_BG = false;
	thread_BG = new thread(&MotorUnion::BGReadWrite, this);
}

void MotorUnion::BGReadWrite()
{
	while (!_is_deleted_thread_BG)
	{
		WriteData();
		ReadData();
		this_thread::sleep_for(chrono::milliseconds(1));
	}
}

void MotorUnion::WriteData() const
{
	// Add parameters
	bool is_Write = false;
	for (int i = 0; i < Motor_Union.size(); i++)
		is_Write |= Motor_Union.at(i)->WriteData();

	// Write to motor
	if (is_Write)
	{
		for (int i = 0; i < groupBulkWrite.size(); i++)
		{
			int dxl_comm_result = groupBulkWrite.at(i)->txPacket();
			if (dxl_comm_result != COMM_SUCCESS)
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			groupBulkWrite.at(i)->clearParam();
		}
	}
}

void MotorUnion::ReadData() const
{
	// Add parameters
	for (int i = 0; i < Motor_Union.size(); i++)
		Motor_Union.at(i)->AddParam();

	// Read Data
	for (int i = 0; i < groupBulkRead.size(); i++)
	{
		int dxl_comm_result = groupBulkRead.at(i)->txRxPacket();
		if (dxl_comm_result != COMM_SUCCESS)
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}

	// Record to Motor
	for (int i = 0; i < Motor_Union.size(); i++)
		Motor_Union.at(i)->ReadData();

	// clear parameters
	for (int i = 0; i < groupBulkRead.size(); i++)
		groupBulkRead.at(i)->clearParam();
}

/// New function ///
int MotorUnion::Motor_ID2idx(int id)
{

	int arr[2] = {12, 16}; // key in current motor id
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		if (arr[i] == id)
		{
			return i;
			break;
		}
	}
	return 0;
}
int MotorUnion::Motor_idx2ID(int idx)
{
	int arr[2] = {12, 16}; // key in current motor id
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		if (i == idx)
		{
			return arr[i];
			break;
		}
	}
	return 0;
}

void MotorUnion::Sync_Drive(int driver_id[], int velocity[])
{
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		SetMotor_Velocity(Motor_ID2idx(driver_id[i]), velocity[i]);
	}
	sleep(1); // cannot delete this line
}
void MotorUnion::AllMotorsTorqueEnableState()
{
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		if (GetMotor_TorqueEnable(i) == true)
		{
			printf("Motor%d is connect\n", Motor_idx2ID(i));
		}
		else
		{
			printf("Motor%d is disconnect\n", Motor_idx2ID(i));
		}
	}
}
void MotorUnion::MotorTorqueEnableState(int id)
{

	if (GetMotor_TorqueEnable(Motor_ID2idx(id)) == true)
	{
		printf("Motor%d is connect\n", Motor_idx2ID(id));
	}
	else
	{
		printf("Motor%d is disconnect\n", Motor_idx2ID(id));
	}
}
int **MotorUnion::AllMotorState()
{
	int **state = new int *[Motor_Union.size()];
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		state[i] = new int[8];
	}
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		state[i][0] = Motor_idx2ID(i);
		state[i][1] = GetMotor_TorqueEnable(i);
		state[i][2] = GetMotor_PresentAngle(i);
		state[i][3] = GetMotor_PresentVelocity(i);
		state[i][4] = GetMotor_PresentCurrent(i);
		state[i][5] = GetMotor_Angle(i);
		state[i][6] = GetMotor_Velocity(i);
		state[i][7] = GetMotor_Accel(i);
	}
	return state;
}
extern "C"
{
	MotorUnion *MotorUnion_new() { return new MotorUnion({1,12, 16,22,26}, {"Pro20","Pro100", "Pro20","Pro100", "Pro20"}); }
	// Set data//
	void SetAllMotorsOperatingMode_new(MotorUnion *MotorUnion, unsigned char mode) { MotorUnion->SetAllMotorsOperatingMode(mode); }
	void SetAllMotorsTorqueEnableOn_new(MotorUnion *MotorUnion) { MotorUnion->SetAllMotorsTorqueEnable(true); }
	void SetAllMotorsTorqueEnableOff_new(MotorUnion *MotorUnion) { MotorUnion->SetAllMotorsTorqueEnable(false); }
	void SetMotor_Velocity_new(MotorUnion *MotorUnion, unsigned char id, float vel) { MotorUnion->SetMotor_Velocity(id, vel); }
	void SetMotor_Angle_new(MotorUnion *MotorUnion, unsigned char id, float angle) { MotorUnion->SetMotor_Angle(id, angle); }
	void SetMotor_Current_new(MotorUnion *MotorUnion, unsigned char id, float current) { MotorUnion->SetMotor_Current(id, current); }
	// Get data//
	int GetMotor_ID_new(MotorUnion *MotorUnion, unsigned char id) { return MotorUnion->GetMotor_ID(id); }
	bool GetMotor_TorqueEnable_new(MotorUnion *MotorUnion, unsigned char id) { return MotorUnion->GetMotor_TorqueEnable(id); }
	int GetMotor_Operating_Mode_new(MotorUnion *MotorUnion, unsigned char id) { return MotorUnion->GetMotor_Operating_Mode(id); }
	float GetMotor_PresentAngle_new(MotorUnion *MotorUnion, unsigned char id) { return MotorUnion->GetMotor_PresentAngle(id); }		  //
	float GetMotor_PresentVelocity_new(MotorUnion *MotorUnion, unsigned char id) { return MotorUnion->GetMotor_PresentVelocity(id); } //
	float GetMotor_PresentCurrent_new(MotorUnion *MotorUnion, unsigned char id) { return MotorUnion->GetMotor_PresentCurrent(id); }	  //
	float GetMotor_Angle_new(MotorUnion *MotorUnion, unsigned char id) { return MotorUnion->GetMotor_Angle(id); }					  //
	float GetMotor_Velocity_new(MotorUnion *MotorUnion, unsigned char id) { return MotorUnion->GetMotor_Velocity(id); }
	float GetMotor_Accel_new(MotorUnion *MotorUnion, unsigned char id) { return MotorUnion->GetMotor_Accel(id); }
	float GetMotor_Current_new(MotorUnion *MotorUnion, unsigned char id) { return MotorUnion->GetMotor_Current(id); }
}