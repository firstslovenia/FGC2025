package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.generic.SlidingWindow;
import org.firstinspires.ftc.teamcode.generic.Vector2D;

public class Drivetrain {

	LinearOpMode callingOpMode;
 	Hardware hardware;

	/// Whether or not to debug values to telemetry
	public boolean debug = true;


	/// Whether or not to make translation field centric via our odometry heading
	public boolean fieldCentricTranslation = true;

	/// Whether or not to make rotation field centric - users input a target rotation on the rotation stick
	public boolean fieldCentricRotation = true;

	/// Motor translation power multiplier
	public double translationMultiplier = 1.0;

	/// Motor rotation power multiplier
	public double rotationMultiplier = 1.0;


	/// Coefficients for PIDF rotation control
	public double rotation_koeficient_p = Math.PI / 6.50;
	public double rotation_koeficient_i = Math.PI / 20.0;
	public double rotation_koeficient_d = 0.0;
	public double rotation_koeficient_f = 0.0;


	/// Internal values to keep track of for PIDF
	double rotation_error_integral = 0.0;
	double rotation_error_previous = 0.0;
	long rotation_error_previous_time = 0;

	/// A sliding window of the last 10 readings of the heading difference
	///
	/// Pair.first is the heading difference in radians
	/// Pair.last is System.currentTimeMillis at the time of the reading
	SlidingWindow<Pair<Float, Long>> last_heading_differences = new SlidingWindow<>(30);

	/// Our rotational speed in radians per second
	double angular_velocity_rad_per_s = 0.0;


	/// The last robot orientation, saved so we can reset it if our readout becomes all zeroes
	Orientation last_robot_orientation;

	/// Saved so we don't try to reset the imu 1000x times a second
	long last_imu_reset = 0;

	public Drivetrain(LinearOpMode opMode, Hardware hw_map) {
		callingOpMode = opMode;
		hardware = hw_map;
		resetStartingDirection();
	}

	/// Sets the starting direction to the current direction
	public void resetStartingDirection() {
		hardware.imu.resetYaw();
	}

	/// Returns how much we've turned (in radians) since the start, with a provided Orientation
	///
	/// What our old odometry heading used to be
	public float getHeadingDifferenceFromStart(Orientation orientation) {
		if (isIMUOk(orientation)) {
			return orientation.thirdAngle;
		} else {
			tryFixIMU();
			return last_robot_orientation.thirdAngle;
		}
	}

	/// Returns whether the IMU orientation sensors are working correctly, with an existing Orientation to avoid calling the expensive reading twice
	public boolean isIMUOk(Orientation orientation) {
		return orientation.thirdAngle != 0.0f;
	}

	/// Resets the IMU after we get BONKed
	public void tryFixIMU() {
		if ((System.currentTimeMillis() - last_imu_reset) > 1000) {
			hardware.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(last_robot_orientation)));
			last_imu_reset = System.currentTimeMillis();
		}
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

		// Save this, so we only call it once (6 ms!!!!) -> slightly expensive
		Orientation current_orientation = hardware.imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

		float heading_difference_from_start = getHeadingDifferenceFromStart(current_orientation);

		Pair<Double, Double> translation_inputs = Drivetrain.getMagnitudeAndPhiFor(translation_stick.x, translation_stick.y);

		double translation_power = translation_inputs.first;
		double translation_direction = translation_inputs.second;

		if (fieldCentricTranslation) {
			translation_direction = translation_direction - heading_difference_from_start;
		}

		translation_power = translation_power * translationMultiplier;

		translation_power = Math.min(translation_power, 1.0);
		translation_power = Math.max(translation_power, 0.0);

		// How much power to use for clockwise rotation, between -1 and 1
		//
		// (-1 for max counterclockwise power)
		double clockwise_rotation_power = 0.0;

		if (fieldCentricRotation) {
			Pair<Double, Double> rotation_inputs = Drivetrain.getMagnitudeAndPhiFor(rotation_stick.x, rotation_stick.y);

			double rotation_power = rotation_inputs.first;
			double wanted_heading = rotation_inputs.second;

			// Our heading has 0 as forward, not as to the right - adjust by 90 degrees
			wanted_heading = wanted_heading - Math.PI / 2;

			if (rotation_power > 0.3) {
				double needed_turn = wanted_heading - heading_difference_from_start;

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

				// Would it be faster to turn in the other direction?
				boolean smaller_turn_other_way = Math.abs(needed_turn_other_way) < Math.abs(needed_turn);

				// If we are already moving in the other direction pretty quickly, don't switch directions
				boolean rotating_quickly = Math.abs(angular_velocity_rad_per_s) >= Math.PI / 2.0;
				boolean rotating_quickly_other_way = rotating_quickly && ((angular_velocity_rad_per_s > 0.0 && needed_turn < 0.0) || (angular_velocity_rad_per_s < 0.0 && needed_turn > 0.0));

				if (smaller_turn_other_way || rotating_quickly_other_way) {
					needed_turn = needed_turn_other_way;
				}

				if (debug) {
					callingOpMode.telemetry.addLine("-- Field centric rotation --");
					callingOpMode.telemetry.addData("wanted heading", Math.toDegrees(wanted_heading));
					callingOpMode.telemetry.addData("needed turn", Math.toDegrees(needed_turn));
					callingOpMode.telemetry.addData("needed turn (other way)", Math.toDegrees(needed_turn_other_way));
					callingOpMode.telemetry.addData("rotating quickly", rotating_quickly);
					callingOpMode.telemetry.addData("rotating quickly (other way)", rotating_quickly_other_way);
				}

				// Don't wobble wobble
				double epsilon = Math.PI / 180;

				if (Math.abs(needed_turn) > epsilon) {

					double rotation_error_derivative = (needed_turn - rotation_error_previous) / (((double) System.currentTimeMillis() - rotation_error_previous_time) / 1000.0);

					clockwise_rotation_power = (rotation_koeficient_p * needed_turn) + (rotation_koeficient_i * rotation_error_integral) + (rotation_koeficient_d * rotation_error_derivative) + rotation_koeficient_f;

					// Our turn direction is flipped -> clockwise (our +) is mathematically negative and vice versa
					clockwise_rotation_power *= -1.0;

					if (debug) {
						callingOpMode.telemetry.addData("proportional", needed_turn * rotation_koeficient_p);
						callingOpMode.telemetry.addData("integral", rotation_error_integral * rotation_koeficient_i);
						callingOpMode.telemetry.addData("derivative", rotation_error_derivative * rotation_koeficient_d);
						callingOpMode.telemetry.addData("constant", rotation_koeficient_f);
					}

					rotation_error_previous = needed_turn;

					if (rotation_error_previous_time != 0.0) {
						rotation_error_integral += needed_turn * ((double) (System.currentTimeMillis() - rotation_error_previous_time) / 1000.0);
					}

					rotation_error_previous_time = System.currentTimeMillis();
				}
				else {
					rotation_error_integral = 0.0;
					rotation_error_previous = 0.0;
					rotation_error_previous_time = 0;
				}
			}

		} else {
			clockwise_rotation_power = rotation_stick.x;
		}

		clockwise_rotation_power = clockwise_rotation_power * rotationMultiplier;

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

		if (last_heading_differences.length() >= 2) {

			Pair<Float, Long> first = last_heading_differences.first().get();
			Pair<Float, Long> last = last_heading_differences.last().get();

			double delta_heading = last.first - first.first;
			double delta_time_ms = last.second - first.second;

			angular_velocity_rad_per_s = delta_heading / (delta_time_ms / 1000.0);

			callingOpMode.telemetry.addData("Last 30 loops took (ms)", delta_time_ms);
		}

		float heading_diff = heading_difference_from_start;
		long heading_diff_time = System.currentTimeMillis();

		last_heading_differences.push(Pair.create(heading_diff, heading_diff_time));

		if (isIMUOk(current_orientation)) {
			last_robot_orientation = current_orientation;

			// This is an expensive call - only do it once
			//last_robot_orientation = hardware.imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
		} else {
			tryFixIMU();
		}

		if (debug) {

			callingOpMode.telemetry.addLine("-- Drivetrain --");
			callingOpMode.telemetry.addData("translation power     ", translation_power);
			callingOpMode.telemetry.addData("translation local  dir", Math.toDegrees(translation_direction));

			if (fieldCentricTranslation) {
				callingOpMode.telemetry.addData("translation global dir",  Math.toDegrees(translation_inputs.second));
			}

			callingOpMode.telemetry.addData("Left  Forward motor power", leftForward);
			callingOpMode.telemetry.addData("Right Forward motor power", rightForward);
			callingOpMode.telemetry.addData("Front Sideways motor power", frontSideways);
			callingOpMode.telemetry.addData("Back  Sideways motor power", backSideways);

			callingOpMode.telemetry.addData("heading difference", Math.toDegrees(heading_diff));
			callingOpMode.telemetry.addData("Angular velocity (rad / s)", angular_velocity_rad_per_s);
			callingOpMode.telemetry.addData("IMU ok", isIMUOk(current_orientation));
			callingOpMode.telemetry.addData("Last IMU fix", last_imu_reset);
		}
	}
}
