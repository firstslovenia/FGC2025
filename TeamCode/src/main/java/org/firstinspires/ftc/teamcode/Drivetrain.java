package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Drivetrain {

	LinearOpMode callingOpmode;
 	Hardware hardware;
	Odometry odometry;

	/// Whether or not to make translation field centric via our odometry heading
	public boolean fieldCentricTranslation = false;

	/// Whether or not to make rotation field centric - users input a target rotation on the rotation stick
	public boolean fieldCentricRotation = false;

	public Drivetrain(LinearOpMode opMode, Hardware hw_map, Odometry odometry) {
		callingOpmode = opMode;
		hardware = hw_map;
		this.odometry = odometry;
	}

	/// Computes (magnitude, phi) from x and y values of a joystick
	///
	/// (x, y in \[-1, 1\])
	///
	/// eg. if we look between phi = 0 and PI / 2:
	///
	/// │xxxxx
	/// │     xxxxx
	/// │         xxx
	/// │           xxxx
	/// │              xxx
	/// │   x            xxxx
	/// ├────────►@         xx
	/// │        /▲          xx
	/// │       / │           xx
	/// │      /  │            xx
	/// │   r /   │             xx
	/// │    /    │ y            xx
	/// │   /+++  │               x
	/// │  /    + │               xx
	/// │ /  phi +│                x
	/// │/       +│                x
	/// └─────────┴─────────────────
	///
	/// We get joystick values x and y, but we'd like to know the angle (phi) and magnitude (how far along we pressed the stick)
	///
	/// ## Why not just use x and y?
	///
	/// Using this approach we know the exact angle we want the robot to go -> we can easily
	/// add our current heading for field-centric movement
	///
	public static Pair<Double, Double> getMagnitudeAndPhiFor(double x, double y) {
		double tan = Double.POSITIVE_INFINITY;

		if (x != 0.0) {
			tan = y / x;
		}

		// This is between - PI / 2 and PI / 2
		double phi = Math.atan(tan);

		// Account for the remaining half of the circle
		if (x < 0.0) {
			phi += Math.PI;
		}

		// Try to have positive angles
		if (phi < 0.0) {
			phi += Math.PI * 2;
		}

		double magnitude = Math.sqrt(x * x + y * y);

		return new Pair<>(magnitude, phi);
	}

	/// Updates the drive motors based off a translation and rotation stick
	public void update(Vector2D translation_stick, Vector2D rotation_stick) {

		Pair<Double, Double> translation_inputs = Drivetrain.getMagnitudeAndPhiFor(translation_stick.x, translation_stick.y);

		// Gamepads like to give us both x and y between 0 and 1, meaning the length can be between 0 and sqrt(2)
		double translation_power = translation_inputs.first / Math.sqrt(2);
		double translation_direction = translation_inputs.second;

		if (fieldCentricTranslation) {
			translation_direction = translation_direction - odometry.heading;
		}

		translation_power = Math.min(translation_power, 1.0);
		translation_power = Math.max(translation_power, 0.0);

		// How much power to use for clockwise rotation, between -1 and 1
		//
		// (-1 for max counterclockwise power)
		double clockwise_rotation_power = 0.0;

		if (fieldCentricRotation) {
			Pair<Double, Double> rotation_inputs = Drivetrain.getMagnitudeAndPhiFor(rotation_stick.x, rotation_stick.y);

			// Gamepads like to give us both x and y between 0 and 1, meaning the length can be between 0 and sqrt(2)
			double rotation_power = rotation_inputs.first / Math.sqrt(2);

			double wanted_heading = rotation_inputs.second;

			// Our heading has 0 as forward, not as to the right - adjust by 90 degrees
			wanted_heading = wanted_heading - Math.PI / 2;

			if (rotation_power > 0.3) {
				double needed_turn = wanted_heading - odometry.heading;

				// Check optimal direction
				// If the difference > 0 -> counter clockwise
				// If the difference < 0 -> clockwise
				// We check if changing the direction (so changing the sign) makes for a small difference to travel
				double needed_turn_other_way;

				if (needed_turn < 0.0) {
					needed_turn_other_way = needed_turn + 2 * Math.PI;
				} else {
					needed_turn_other_way = needed_turn - 2 * Math.PI;
				}

				if (Math.abs(needed_turn_other_way) < Math.abs(needed_turn)) {
					needed_turn = needed_turn_other_way;
				}

				// Only turn if the needed turn is > 1 degree
				double epsilon = Math.PI / 180;

				if (Math.abs(needed_turn) > epsilon) {
					if (needed_turn > 0) {
						// Our turn direction needs to be positive, meaning counterclockwise
						clockwise_rotation_power = -1.0;
					} else {
						// Our turn direction needs to be negative, meaning clockwise
						clockwise_rotation_power = 1.0;
					}
				}
			}

		} else {
			clockwise_rotation_power = rotation_stick.x;
		}

		clockwise_rotation_power = Math.min(clockwise_rotation_power, 1.0);
		clockwise_rotation_power = Math.max(clockwise_rotation_power, -1.0);

		double sin_phi = Math.sin(translation_direction);
		double cos_phi = Math.cos(translation_direction);

		double leftForward = (sin_phi * translation_power) + clockwise_rotation_power;
		double rightForward = (sin_phi * translation_power) - clockwise_rotation_power;
		double frontSideways = (cos_phi * translation_power) + clockwise_rotation_power;
		double backSideways = (cos_phi * translation_power) - clockwise_rotation_power;

		// Normalize all of them to get the expected result
		double maxPower = Math.max(Math.max(Math.abs(leftForward), Math.abs(rightForward)), Math.max(Math.abs(frontSideways), Math.abs(backSideways)));

		if (maxPower > 1.0) {
			leftForward /= maxPower;
			rightForward /= maxPower;
			frontSideways /= maxPower;
			backSideways /= maxPower;
		}

		hardware.leftForwardMotor.setPower(leftForward);
		hardware.rightForwardMotor.setPower(rightForward);
		hardware.frontSidewaysMotor.setPower(frontSideways);
		hardware.backSidewaysMotor.setPower(backSideways);
	}
}
