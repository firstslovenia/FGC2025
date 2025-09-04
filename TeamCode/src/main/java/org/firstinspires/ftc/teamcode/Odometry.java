package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.generic.Vector2D;

/// Implementation of odometry using only the drive motor encoders, assuming they don't slide
///
/// This is used primary to keep track of our heading for field centric movement and rotation
public class Odometry {

	LinearOpMode callingOpMode = null;
	Hardware hardware;

	/// Whether or not to debug values to telemetry
	public boolean debug = true;

	// Calibration values
	// --------------------
	/// How many steps (hopefully same for all motors) are needed to move the robot 1 meter forward
	///
	/// To measure this, drive the robot some distance straight forward, then divide steps with meters travelled
	static int magicStepsForOneMeter = 10;

	/// How many steps are needed (hopefully same for all motors) are needed to rotate the robot one full circle (2 Pi radians)
	///
	/// To measure this, try to rotate the robot exactly one whole circle (maybe: attach a laser pointer?)
	static int magicStepsForTwoPi = 4208;
	// --------------------

	// Static values for calculating based off calibration values
	static double metersPerStep = 1.0 / magicStepsForOneMeter;

	static double radsPerStep = 2 * Math.PI / magicStepsForTwoPi;

	// Motor directions - how our wheels turn (relative to +y = forward and +x = right) in relation to our motors
	Vector2D frontSidewaysVector = new Vector2D(1, 0);
	Vector2D rightForwardVector = new Vector2D(0, 1);
	Vector2D leftForwardVector = new Vector2D(0, 1);
	Vector2D backSidewaysVector = new Vector2D(1, 0);

	/// A 2d vector of our position, relative to where we started
	///
	/// 1 unit = 1 meter
	public Vector2D position = new Vector2D();

	/// How much our robot has rotated since the beginning, in radians
	///
	/// Element of [0, 2 * Pi)
	public Double heading = 0.0;

	// The positions of our motor encoders at our last update
	int lastFrontSidewaysPosition = 0;
	int lastRightForwardPosition = 0;
	int lastLeftForwardPosition = 0;
	int lastBackSidewaysPosition = 0;

	public Odometry(LinearOpMode opmode, Hardware hardware) {
		this.hardware = hardware;
		callingOpMode = opmode;
	}

	/// Updates our position and heading assuming our motors turned some steps
	public void updateForStepDelta(int frontSidewaysDelta, int rightForwardDelta, int leftForwardDelta, int backSidewaysDelta) {
		Vector2D front_side_steps = frontSidewaysVector.mul_by(frontSidewaysDelta);
		Vector2D back_side_steps = backSidewaysVector.mul_by(backSidewaysDelta);
		Vector2D right_fwd_steps = rightForwardVector.mul_by(rightForwardDelta);
		Vector2D left_fwd_steps = leftForwardVector.mul_by(leftForwardDelta);

		// Vectors added together -> local translation
		Vector2D translation_steps = front_side_steps.add(back_side_steps).add(right_fwd_steps).add(left_fwd_steps);

		// Account for our heading: local -> global
		translation_steps = translation_steps.rotateCCWFor(heading);

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
		Double rotation_steps = (double) (leftForwardDelta + frontSidewaysDelta - rightForwardDelta - backSidewaysDelta);
		Double rotation_clockwise_delta = rotation_steps * radsPerStep;

		// Unit circle rotation is counterclockwise
		heading = (heading - rotation_clockwise_delta) % (2 * Math.PI);
	}

	/// Updates our position and heading based off the current motor positions and the ones from the last update
	public void update() {

		int frontSidewaysPosition = hardware.frontSidewaysMotor.getCurrentPosition();
		int backSidewaysPosition = hardware.backSidewaysMotor.getCurrentPosition();
		int rightForwardPosition = hardware.rightForwardMotor.getCurrentPosition();
		int leftForwardPosition = hardware.leftForwardMotor.getCurrentPosition();

		int frontSidewaysDelta = frontSidewaysPosition - lastFrontSidewaysPosition;
		int backSidewaysDelta = backSidewaysPosition - lastBackSidewaysPosition;
		int rightForwardDelta = rightForwardPosition - lastRightForwardPosition;
		int leftForwardDelta = leftForwardPosition - lastLeftForwardPosition;

		updateForStepDelta(frontSidewaysDelta, rightForwardDelta, leftForwardDelta, backSidewaysDelta);

		if (debug) {
			callingOpMode.telemetry.addLine("-- Odometry --");
			callingOpMode.telemetry.addData("heading (rads)", heading);
			callingOpMode.telemetry.addData("heading (deg)", Math.toDegrees(heading));
			callingOpMode.telemetry.addData("position (x)", position.x);
			callingOpMode.telemetry.addData("position (y)", position.x);
			callingOpMode.telemetry.addData("n  front sideways", frontSidewaysPosition);
			callingOpMode.telemetry.addData("n  back  sideways", backSidewaysPosition);
			callingOpMode.telemetry.addData("n  right forward ", rightForwardPosition);
			callingOpMode.telemetry.addData("n  left  forward ", leftForwardPosition);
			callingOpMode.telemetry.addData("Δn front sideways", frontSidewaysDelta);
			callingOpMode.telemetry.addData("Δn back  sideways", backSidewaysDelta);
			callingOpMode.telemetry.addData("Δn right forward ", rightForwardDelta);
			callingOpMode.telemetry.addData("Δn left  forward ", leftForwardDelta);
		}

		lastFrontSidewaysPosition = frontSidewaysPosition;
		lastBackSidewaysPosition = backSidewaysPosition;
		lastRightForwardPosition = rightForwardPosition;
		lastLeftForwardPosition = leftForwardPosition;
	}
}
