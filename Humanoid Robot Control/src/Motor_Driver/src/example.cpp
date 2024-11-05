#include <MotorUnion.h>
#include <unistd.h> /* UNIX standard function definitions */
#include <termio.h>
#include <stdio.h>

using namespace std;

int main(int argc, char *argv[])
{
    MotorUnion motor_driver({12, 16}, {"Pro100", "Pro20"}); // Mx106 Vmax 210  //Mx28  Vmax 230
    int motor_id[2] = {12, 16};
    // int motor_vel[2] = {300, 300};
    int motor_vel[2] = {0, 0};
    motor_driver.SetAllMotorsOperatingMode(0); // need to set mode on wizard 2.0 ! cannot delete this line!
    motor_driver.AllMotorsTorqueEnableState();
    motor_driver.SetAllMotorsTorqueEnable(true);
    motor_driver.AllMotorsTorqueEnableState();
    // motor_driver.SetMotor_Velocity(0, 0);
    motor_driver.SetMotor_Current(1, -150.2);
    sleep(2);
    float a = motor_driver.GetMotor_PresentCurrent(1);
    float b = motor_driver.GetMotor_Current(1);
    cout << a << " " << b << endl;
    // motor_driver.SetMotor_Current(1, 50.3);
    // sleep(2);
    // motor_driver.SetMotor_Current(1, 2000);
    sleep(2);

    motor_driver.SetMotor_Current(1, 100);
    sleep(2);
    // motor_driver.SetAllMotorsTorqueEnable(false);
}
// motor_driver.SetMotor_Velocity(0, 0);
// motor_driver.SetMotor_Velocity(1, 0);
// sleep(1);不知所以...
//////////////////code using MotorUnion.h/////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// MotorUnion motor_driver({0,6},{"Mx28","Mx28"});
//  motor_driver.SetAllMotorsOperatingMode(1);         //need to set mode on wizard 2.0 !!!!!!!!!!!!!!!!!!!!!!!!!!
//  usleep(1000);

// motor_driver.SetMotor_TorqueEnable(0, true) ;
//  usleep(1000);
// motor_driver.SetMotor_TorqueEnable(1, true) ;
//  usleep(1000);
// motor_driver.SetMotor_Velocity(0,  200);
// usleep(5000);
// motor_driver.SetMotor_Velocity(1,  200);
// sleep(2);
//  motor_driver.SetMotor_Velocity(0, 0);
//  usleep(5000);
// motor_driver.SetMotor_Velocity(1, 0);
// sleep(3);
// motor_driver.SetMotor_TorqueEnable(0,false ) ;
// motor_driver.SetMotor_TorqueEnable(1,false ) ;
// int *ControlButton()
// {
//     int *array = new int[2];
//     array[0] = scanKeyboard();
//     array[1] = scanKeyboard();
//     //printf("%d\n",array);
//     return array;
// }