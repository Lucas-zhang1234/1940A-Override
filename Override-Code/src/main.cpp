#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "robot.hpp"
#include "macros.hpp"
#include "macro_manager.hpp"
#include "position_control.hpp"
#include <chrono>

namespace {
constexpr double kArmGearRatio = 5.0;
constexpr double kWristGearRatio = 2.0;
constexpr double kWristToArmMotorScale = kWristGearRatio / kArmGearRatio;
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	// Since the front motors of each side are green:
    Left_MG.set_gearing(pros::MotorGearset::green, 0);
    Right_MG.set_gearing(pros::MotorGearset::green, 0);

	Arm.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
	Lift.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
	Wrist.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);

	Arm.set_encoder_units(pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_DEGREES);
	Lift.set_encoder_units(pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_DEGREES);
	Wrist.set_encoder_units(pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_DEGREES);

	

	position_control::start();
	pros::Task macroManagerTask(macroTask, nullptr, "Macro Manager Task");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() 
{
	Arm.set_zero_position(0);
	Lift.set_zero_position(0);
	Wrist.set_zero_position(0);
	// pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Initial Arm Position: %f", Arm.get_position());
	// auto start = std::chrono::high_resolution_clock::now();
	// position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, 360, 1200, 2000);
	// auto end = std::chrono::high_resolution_clock::now();
	// auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	// pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Final Arm Position: %f", Arm.get_position());
	// pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Duration: %lld ms", duration.count());
	Fingers.retract();
	pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Initial Lift Position: %f", Lift.get_position());
	auto start = std::chrono::high_resolution_clock::now();
	position_control::move_absolute_degrees_blocking(position_control::MotorId::Lift, 180, 300, 2000);
	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Final Lift Position: %f", Lift.get_position());
	pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Duration: %lld ms", duration.count());
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	double lastArmPosition = Arm.get_position();
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = Master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = Master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		Left_MG.move(dir + turn);                      // Sets left motor voltage
		Right_MG.move(dir - turn);                     // Sets right motor voltage

		if (isMacroRunning())
		{
			if (Partner.get_digital(pros::E_CONTROLLER_DIGITAL_X))
			{
				clearMacros();
			}
			continue;
		}
		if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			// intake out
			Intake.move_voltage(12000);
		}
		else if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			// intake in
			Intake.move_voltage(-12000);
		}
		else
		{
			Intake.brake();
		}

		if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			Lift.move_voltage(12000);
		}
		else if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			Lift.move_voltage(-12000);
		}
		else
		{
			Lift.brake();
		}

		if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
		{
			Arm.move_voltage(-12000);
		} 
		else if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
		{
			Arm.move_voltage(12000);
		}
		else 
		{
			Arm.brake();
		}

		const double armPosition = Arm.get_position();
		const double armDelta = armPosition - lastArmPosition;
		const double wristDelta = -armDelta * kWristToArmMotorScale;
		Wrist.move_relative(wristDelta, 200);
		lastArmPosition = armPosition;

		if (Master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
		{
			// macro to move intake back far enough, grab the pin with the claw, and rotate it upright
			tryAddMacroToQueue(Macro::GRAB_PIN);
		}

		if (Master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
		{
			// macro to move intake back far enough, grab the pin with the claw, and rotate it upright
			tryAddMacroToQueue(Macro::SCORE_POSITION);
		}

		if (Master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
		{
			Fingers.toggle();
		}

		pros::delay(20);                               // Run for 20 ms then update
	}
}