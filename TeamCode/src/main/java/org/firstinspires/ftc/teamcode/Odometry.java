package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/// Implementation of odometry using only the drive motor encoders, assuming they don't slide
///
/// This is used primary to keep track of our heading for field centric movement and rotation
public class Odometry {
	Hardware hardware;

	// Calibration values
	// --------------------
	/// How many steps (hopefully same for all motors) are needed to move the robot 1 meter forward
	///
	/// To measure this, drive the robot some distance straight forward, then divide steps with meters travelled
	static int magicStepsForOneMeter = 10;

	/// How many steps are needed (hopefully same for all motors) are needed to rotate the robot one full circle (2 Pi radians)
	///
	/// To measure this, try to rotate the robot exactly one whole circle (maybe: attach a laser pointer?)
	static int magicStepsForTwoPi = 10;
	// --------------------

	// Static values for calculating based off calibration values
	static double metersPerStep = 1.0 / magicStepsForOneMeter;

	static double radsPerStep = 2 * Math.PI / magicStepsForTwoPi;

	// Motor directions - how our wheels turn (relative to +y = forward and +x = right) in relation to our motors
	Vector2D leftFrontVector = new Vector2D(1, 0);
	Vector2D rightFrontVector = new Vector2D(0, 1);
	Vector2D leftBackVector = new Vector2D(0, 1);
	Vector2D rightBackVector = new Vector2D(1, 0);

	/// A 2d vector of our position, relative to where we started
	///
	/// 1 unit = 1 meter
	Vector2D position = new Vector2D();

	/// Radian heading of our robot in a unit circle
	///
	/// Element of [0, 2 * Pi)
	Double heading = 0.0;

	// The positions of our motor encoders at our last update
	int lastLeftFrontPosition = 0;
	int lastRightFrontPosition = 0;
	int lastLeftBackPosition = 0;
	int lastRightBackPosition = 0;

	public Odometry(Hardware hardware) {
		this.hardware = hardware;
	}

	/// Updates our position and heading assuming our motors turned some steps
	public void updateForStepDelta(int leftFrontDelta, int rightFrontDelta, int leftBackDelta, int rightBackDelta) {
		Vector2D left_front_steps = leftFrontVector.mul_by(leftFrontDelta);
		Vector2D right_front_steps = rightFrontVector.mul_by(rightFrontDelta);
		Vector2D left_back_steps = leftBackVector.mul_by(leftBackDelta);
		Vector2D right_back_steps = rightBackVector.mul_by(rightBackDelta);

		// Vectors added together -> translation
		Vector2D translation_steps = left_front_steps.add(right_front_steps).add(left_back_steps).add(right_back_steps);
		Vector2D translation_meters = translation_steps.mul_by(metersPerStep);

		position = position.add(translation_meters);

		// Clockwise (negative) rotation
		// Similar to the drivetrain code
		//  ╔════════════════════╗
		//  ║ <-->             ⌃ ║
		//  ║  -> (+)   (-)  ⌄ | ║
		//  ║                  ⌄ ║
		//  ║                    ║
		//  ║ ⌃                  ║
		//  ║ | ⌃ (+)   (-)  <-  ║
		//  ║ ⌄             <--> ║
		//  ╚════════════════════╝
		Double rotation_steps = (double) (leftBackDelta + leftFrontDelta - rightFrontDelta - rightBackDelta);
		Double rotation_clockwise_delta = rotation_steps * radsPerStep;

		// Unit circle rotation is counterclockwise
		heading = heading - rotation_clockwise_delta;
	}

	/// Updates our position and heading based off the current motor positions and the ones from the last update
	public void update() {

		int leftFrontPosition = hardware.leftFrontMotor.getCurrentPosition();
		int rightFrontPosition = hardware.rightFrontMotor.getCurrentPosition();
		int leftBackPosition = hardware.leftBackMotor.getCurrentPosition();
		int rightBackPosition = hardware.rightBackMotor.getCurrentPosition();

		int leftFrontDelta = leftFrontPosition - lastLeftFrontPosition;
		int rightFrontDelta = rightFrontPosition - lastRightFrontPosition;
		int leftBackDelta = leftBackPosition - lastLeftBackPosition;
		int rightBackDelta = rightBackPosition - lastRightBackPosition;

		updateForStepDelta(leftFrontDelta, rightFrontDelta, leftBackDelta, rightBackDelta);

		lastLeftFrontPosition = leftFrontPosition;
		lastRightFrontPosition = rightFrontPosition;
		lastLeftBackPosition = leftBackPosition;
		lastRightBackPosition = rightBackPosition;
	}
}
