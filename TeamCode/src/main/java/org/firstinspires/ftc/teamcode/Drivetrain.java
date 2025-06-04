package org.firstinspires.ftc.teamcode;

import android.util.Pair;

public class Drivetrain {

	private Hardware hardware_map;

	public Drivetrain(Hardware hw_map) {
		hardware_map = hw_map;
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

	/// Updates the drive motors to go with \[0, 1\] power in the (relative) direction phi radians while turning with power \[-1, 1\] clockwise
	public void update(double power, double direction, double turn) {

		power = Math.min(power, 1.0);
		power = Math.max(power, 0.0);

		turn = Math.min(turn, 1.0);
		turn = Math.max(turn, -1.0);

		double sin_phi = Math.sin(direction);
		double cos_phi = Math.cos(direction);

		double leftBack = (sin_phi * power) + turn;
		double rightFront = (sin_phi * power) - turn;
		double leftFront = (cos_phi * power) + turn;
		double rightBack = (cos_phi * power) - turn;

		// Normalize all of them to get the expected result
		double maxPower = Math.max(Math.max(Math.abs(leftBack), Math.abs(rightFront)), Math.max(Math.abs(leftFront), Math.abs(rightBack)));

		if (maxPower > 1.0) {
			leftBack /= maxPower;
			rightFront /= maxPower;
			leftFront /= maxPower;
			rightBack /= maxPower;
		}

		hardware_map.leftBackMotor.setPower(leftBack);
		hardware_map.rightFrontMotor.setPower(rightFront);
		hardware_map.leftFrontMotor.setPower(leftFront);
		hardware_map.rightBackMotor.setPower(rightBack);
	}
}
