#include "../include/main.h"
#include "lemlib/api.hpp"

// 10 & 2 having issues

extern pros::Controller Master;
extern pros::Controller Partner;
// left middle and right back r broken
extern pros::MotorGroup Left_MG;    
extern pros::MotorGroup Right_MG; // pos or neg

extern lemlib::Drivetrain Drivetrain;

extern pros::Imu IMU;

#pragma region Odometry

extern pros::Rotation Vertical_Rot; 
extern pros::Rotation Right_Horizontal_Rot;
extern pros::Rotation Left_Horizontal_Rot;

// fix
extern lemlib::TrackingWheel Vertical_TW;
extern lemlib::TrackingWheel Right_Horizontal_TW;
extern lemlib::TrackingWheel Left_Horizontal_TW;

#pragma endregion

extern lemlib::OdomSensors Sensors;

#pragma region Controllers

// lateral PID controller
extern lemlib::ControllerSettings Lateral_Controller;

// angular PID controller
extern lemlib::ControllerSettings Angular_Controller;

#pragma endregion Controllers

// create the chassis
extern lemlib::Chassis Chassis;

#pragma region Scoring

extern pros::Motor Intake; // check motor gears, neg. or pos. asw
extern pros::Motor Claw_Rotation; // neg. or pos.
extern pros::Motor Arm; // neg. or pos.
extern pros::Motor Lift; // neg. or pos.

extern pros::adi::Pneumatics Claw_Grip;            // Starts retracted, extends when the ADI port is high

#pragma endregion

#pragma region Distance Sensors

extern pros::Distance Back_DS;
extern pros::Distance Right_DS;
extern pros::Distance Left_DS;

#pragma endregion