#include "robot.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"

pros::Controller Master(pros::E_CONTROLLER_MASTER);
pros::Controller Partner(pros::E_CONTROLLER_PARTNER);

pros::MotorGroup Left_MG({-5, -3, -4}, pros::MotorGearset::blue);    
pros::MotorGroup Right_MG({7, 9, 10}, pros::MotorGearset::blue);

lemlib::Drivetrain Drivetrain(&Left_MG, &Right_MG,
                              11.42, // track width in inches
                              lemlib::Omniwheel::NEW_325, // Wheel configuration
                              600, // RPM
                              2 // Drift was 2  0.5
);

pros::Imu IMU(15);

#pragma region Odometry

pros::Rotation Vertical_Rot(6);
pros::Rotation Right_Horizontal_Rot(8);
pros::Rotation Left_Horizontal_Rot(2);

lemlib::TrackingWheel Vertical_TW(&Vertical_Rot, lemlib::Omniwheel::NEW_2, 0.75);
lemlib::TrackingWheel Right_Horizontal_TW(&Right_Horizontal_Rot, lemlib::Omniwheel::NEW_2, 2.9);
lemlib::TrackingWheel Left_Horizontal_TW(&Left_Horizontal_Rot, lemlib::Omniwheel::NEW_2, 2.9);

#pragma endregion

lemlib::OdomSensors Sensors(&Vertical_TW, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &Right_Horizontal_TW, // horizontal tracking wheel 1
                            &Left_Horizontal_TW, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &IMU // inertial sensor
);

#pragma region Controllers

// lateral PID controller
lemlib::ControllerSettings Lateral_Controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              55, // derivative gain (kD)
                                              0.9, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              300, // large error range timeout, in milliseconds
                                              10 // maximum acceleration (slew) - smooths out sudden lurches
);

// angular PID controller
lemlib::ControllerSettings Angular_Controller(2.3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              15, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              200, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

#pragma endregion Controllers

// create the chassis
lemlib::Chassis Chassis(Drivetrain, // drivetrain settings
                        Lateral_Controller, // lateral PID settings
                        Angular_Controller, // angular PID settings
                        Sensors // odometry sensors
);

#pragma region Scoring

pros::Motor Intake(21, pros::v5::MotorGears::blue); // check motor gears, neg. or pos. asw
pros::Motor Wrist(-14, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees); // neg. or pos.
pros::Motor Arm(13, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees); // neg. or pos.
pros::Motor Lift(12, pros::v5::MotorGears::green); // neg. or pos.

pros::adi::Pneumatics Fingers({22, 'A'}, true); // starts closed

#pragma endregion

#pragma region Distance Sensors

pros::Distance Back_DS(17);
pros::Distance Right_DS(16);
pros::Distance Left_DS(20);

#pragma endregion