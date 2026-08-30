#include "robot.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"

pros::Controller Master(pros::E_CONTROLLER_MASTER);
pros::Controller Partner(pros::E_CONTROLLER_PARTNER);

pros::MotorGroup Left_MG({-4, -2, -3}, pros::MotorGearset::blue);    
pros::MotorGroup Right_MG({5, 8, 10}, pros::MotorGearset::blue);

lemlib::Drivetrain Drivetrain(&Left_MG, &Right_MG,
                              11.42, // track width in inches
                              lemlib::Omniwheel::NEW_325, // Wheel configuration
                              600, // RPM
                              2 // Drift was 2  0.5
);

pros::Imu IMU(15);

#pragma region Odometry

pros::Rotation Vertical_Rot(-7);
pros::Rotation Right_Horizontal_Rot(6);

lemlib::TrackingWheel Vertical_TW(&Vertical_Rot, lemlib::Omniwheel::NEW_2, 0.75);
lemlib::TrackingWheel Right_Horizontal_TW(&Right_Horizontal_Rot, lemlib::Omniwheel::NEW_2, 2.9);

#pragma endregion

lemlib::OdomSensors Sensors(&Vertical_TW, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &Right_Horizontal_TW, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &IMU // inertial sensor
);

#pragma region Controllers

// lateral PID controller
lemlib::ControllerSettings Lateral_Controller(1.5472, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0.9, // anti windup (0.9)
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              200, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew) - smooths out sudden lurches (10)
);

// angular PID controller
lemlib::ControllerSettings Angular_Controller(6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3.2, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in degrees (1)
                                              100, // small error range timeout, in milliseconds (100)
                                              3, // large error range, in degrees (3)
                                              200, // large error range timeout, in milliseconds (200)
                                              0 // maximum acceleration (slew) (20)
);

#pragma endregion Controllers

// create the chassis
lemlib::Chassis chassis(Drivetrain, // drivetrain settings
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
pros::Distance Left_DS(19);

#pragma endregion