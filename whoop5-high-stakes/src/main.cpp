#include "main.h"
#include "rev/rev.hh"
#include "globals.hh"
#include <string>

using namespace rev;
using namespace std;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();

	chassis = std::make_shared<rev::SkidSteerChassis>(left_motor_group, right_motor_group);
	odom = std::make_shared<rev::TwoRotationInertialOdometry>(
    fwd,      // The forward sensor
    lat,      // The rightward sensor 
    imu,      // Inertial sensor 
    WHEEL_DIAMETER,  // Diameter of forward wheel
    WHEEL_DIAMETER,  // Diameter of sideways wheel
    FORWARD_WHEEL_OFFSET,  // How far to the right of the center of the robot the forward wheel is
    LATERAL_WHEEL_OFFSET    // How far to the rear of the robot the lateral wheel is from the center
  );

  turn = std::make_shared<CampbellTurn>(chassis, odom, TURN_IKP1, TURN_IKP2);
  reckless = std::make_shared<Reckless>(chassis, odom);

  odom_runner = std::make_shared<rev::AsyncRunner>(odom);
	reckless_runner = std::make_shared<rev::AsyncRunner>(reckless);
	turn_runner = std::make_shared<rev::AsyncRunner> (turn);

	pros::lcd::initialize();

  pros::delay(2000);
  odom->reset_position();

  
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
void competition_initialize() {

}

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
void autonomous() {
	/*
	 * The following function call will drive the robot to the point 20_in, 0_in. The 0_deg is meaningless,
	 * but it is still required otherwise there will be a compiler error. 
	 * The code can be copy and pasted, you just have to change out the coordinates to go to a different spot. 
	*/
	odomHydraulic.set_value(ODOM_DOWN);
	pros::lcd::set_text(1, std::to_string(odom -> get_state().pos.x.convert(inch)));
	pros::lcd::set_text(2, std::to_string(odom -> get_state().pos.y.convert(inch)));
	pros::lcd::set_text(3, std::to_string(odom -> get_state().pos.theta.convert(degree)));


	reckless->go(RecklessPath().with_segment(
		RecklessPathSegment(
			std::make_shared<ConstantMotion>(0.2),
			std::make_shared<PilonsCorrection>(4, 0.3_in),
			std::make_shared<SimpleStop>(0.03_s, 0.15_s, 0.3), 
			{ 1_in, 0_in, 0_deg }, 0_in)
	));
	reckless->await(); // don't run the next code until the robot has reached the target

	// turn the robot 90 degrees
	turn->turn_to_target_absolute(0.7, 90_deg);
	turn->await();



	reckless->go(RecklessPath().with_segment(
		RecklessPathSegment(
			std::make_shared<ConstantMotion>(0.5),
			std::make_shared<PilonsCorrection>(4, 0.3_in),
			std::make_shared<SimpleStop>(0.03_s, 0.15_s, 0.3), 
			{ 20_in, 20_in, 0_deg }, 0_in)
	));
	reckless->await();


	turn->turn_to_target_absolute(0.7, 180_deg);
	turn->await();




	reckless->go(RecklessPath().with_segment(
		RecklessPathSegment(
			std::make_shared<ConstantMotion>(0.5),
			std::make_shared<PilonsCorrection>(4, 0.3_in),
			std::make_shared<SimpleStop>(0.03_s, 0.15_s, 0.3), 
			{ 0_in, 20_in, 0_deg }, 0_in)
	));
	reckless->await();

	turn->turn_to_target_absolute(0.7, 270_deg);
	turn->await();





	reckless->go(RecklessPath().with_segment(
		RecklessPathSegment(
			std::make_shared<ConstantMotion>(0.5),
			std::make_shared<PilonsCorrection>(4, 0.3_in),
			std::make_shared<SimpleStop>(0.03_s, 0.15_s, 0.3), 
			{ 0_in, 0_in, 0_deg }, 0_in)
	));
	reckless->await();

	turn->turn_to_target_absolute(0.7, 0_deg);
	turn->await();
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
	pros::lcd::set_text(0, "Hello PROS User!");
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	odomHydraulic.set_value(ODOM_UP);

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		
		int rightX = master.get_analog(ANALOG_RIGHT_X);
		int leftY = master.get_analog(ANALOG_LEFT_Y);
		

		left_motor_group = leftY+rightX;
		right_motor_group = leftY-rightX;

		pros::delay(20);
	}
}
