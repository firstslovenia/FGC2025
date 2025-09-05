package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.generic.GenericPIDController;
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

	/// Whether or not to try to keep the heading (i. e. to always rotate towards wanted_heading, even if we aren't pressing the stick)
	public boolean keepHeading = true;

	/// Motor translation power multiplier
	public double translationMultiplier = 1.0;

	/// Motor rotation power multiplier
	public double rotationMultiplier = 1.0;

	/// What motor power to consider not actually moving
	///
	/// 2025/09/04 - our motors need to be 0.5 to actually cause **any** movement! For rotation about 0.25 for all motors
	public double power_epsilon = 0.15;

	/// How many ms to wait before releasing the breaks after not moving
	public long stop_breaking_after_ms = 1000;

	/// PID implementation for our rotation motors
	GenericPIDController rotation_pid_controller;

	/// Our rotational speed in radians per second
	double angular_velocity_rad_per_s = 0.0;

	/// What absolute heading we want to turn towards
	///
	/// In radians
	///
	/// By default, this is forward (Pi / 2)
	double wanted_heading = Math.PI / 2.0;

	/// The last robot orientation, saved so we can reset it if our readout becomes all zeroes
	Orientation last_robot_orientation;

	/// Saved so we don't try to reset the imu 1000x times a second
	long last_imu_reset_time = 0;

	/// The millis time when we stopped moving
	long stopped_moving_time = 0;

	/// If our rotation was caused by a robot-centric manual rotation, when we started doing that
	long started_rotating_due_to_manual_stick_time = 0;

	public Drivetrain(LinearOpMode opMode, Hardware hw_map) {
		callingOpMode = opMode;
		hardware = hw_map;
		rotation_pid_controller = new GenericPIDController(callingOpMode, 0.7, 0.0, 0.15, 0.2);
		resetStartingDirection();
	}

	/// Disables the drivetrain and sets the motors to consume the least power
	///
	/// To enable, just start calling update()
	public void disable() {
		hardware.leftForwardMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		hardware.rightForwardMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		hardware.frontSidewaysMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		hardware.backSidewaysMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		hardware.leftForwardMotor.setPower(0);
		hardware.rightForwardMotor.setPower(0);
		hardware.frontSidewaysMotor.setPower(0);
		hardware.backSidewaysMotor.setPower(0);
	}

	/// Sets the starting direction to the current direction
	public void resetStartingDirection() {
		wanted_heading = Math.PI / 2.0;
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
		if ((System.currentTimeMillis() - last_imu_reset_time) > 1000) {
			hardware.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(last_robot_orientation)));
			last_imu_reset_time = System.currentTimeMillis();
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

		if (x == 0.0) {
			if (y >= 0.0) {
				return new Pair<>(y, Math.PI / 2.0);
			}
			else {
				return new Pair<>(-y, - Math.PI / 2.0);
			}
		}

		double tan = y / x;

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
		angular_velocity_rad_per_s = hardware.imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;

		float heading_difference_from_start = getHeadingDifferenceFromStart(current_orientation);

		// Clear rotation due to manual stick; the time is here so we don't stop it 1 ms later, when we're not actually rotating yet
		if (started_rotating_due_to_manual_stick_time != 0 &&
			System.currentTimeMillis() - started_rotating_due_to_manual_stick_time > 500 &&
			angular_velocity_rad_per_s < 0.1
		) {
			started_rotating_due_to_manual_stick_time = 0;
		}

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

		boolean rotating_due_to_manual_stick = started_rotating_due_to_manual_stick_time != 0 || Math.abs(rotation_stick.x) > power_epsilon;
		boolean keep_heading_for_robot_centric = keepHeading && !rotating_due_to_manual_stick;

		if (fieldCentricRotation || keep_heading_for_robot_centric) {

			// Whether or not we'll actively try to rotate this loop
			//
			// This is true if either:
			// - we have a rotational input we want to go towards
			// - we want to always keep our heading
			boolean should_rotate = keepHeading;

			if (fieldCentricRotation) {
				Pair<Double, Double> rotation_inputs = Drivetrain.getMagnitudeAndPhiFor(rotation_stick.x, rotation_stick.y);

				double rotation_power_input = rotation_inputs.first;
				double wanted_heading_input = rotation_inputs.second;

				if (rotation_power_input > 2 * power_epsilon) {
					wanted_heading = wanted_heading_input;
					should_rotate = true;
				}
			}

			if (should_rotate) {

				// Our heading has 0 as forward, not as to the right - adjust by 90 degrees
				double wanted_heading_local = wanted_heading - Math.PI / 2;

				double needed_turn = wanted_heading_local - heading_difference_from_start;

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

				if (smaller_turn_other_way) {
					needed_turn = needed_turn_other_way;
				}

				// Note: this practically didn't work how we wanted it to, and caused other problems.

				// Check for rotating quickly, if we want to switch directions stop first
				/*boolean rotating_quickly = Math.abs(angular_velocity_rad_per_s) >= Math.PI;

				boolean rotating_quickly_other_way = rotating_quickly && ((angular_velocity_rad_per_s > 0.0 && needed_turn < 0.0) || (angular_velocity_rad_per_s < 0.0 && needed_turn > 0.0)) && Math.abs(needed_turn) >= Math.PI / 4;

				// Stop first
				if (rotating_quickly_other_way) {
					clockwise_rotation_power = 0.0;
					needed_turn = 0.0;

					// Don't average stuff out now
					last_wanted_headings = new SlidingWindow(10, last_wanted_headings.last().get());
				}*/

				if (debug) {
					callingOpMode.telemetry.addLine("-- Rotational PIDF --");
					callingOpMode.telemetry.addData("wanted heading", Math.toDegrees(wanted_heading));
					callingOpMode.telemetry.addData("needed turn", Math.toDegrees(needed_turn));
					callingOpMode.telemetry.addData("needed turn (other way)", Math.toDegrees(needed_turn_other_way));
				}

				// Don't wobble wobble
				double minimum = (Math.PI / 180.0) * 2.0;

				if (Math.abs(needed_turn) > minimum) {

					rotation_pid_controller.error = needed_turn;
					rotation_pid_controller.update();
					clockwise_rotation_power = rotation_pid_controller.output;

					// Our turn direction is flipped -> clockwise (our +) is mathematically negative and vice versa
					clockwise_rotation_power *= -1.0;
				} else {
					rotation_pid_controller.reset();
				}
			} else {
				rotation_pid_controller.reset();
			}

		} else {
			if (Math.abs(rotation_stick.x) > power_epsilon) {
				started_rotating_due_to_manual_stick_time = System.currentTimeMillis();
			}

			clockwise_rotation_power = rotation_stick.x;

			// For keep heading to work; don't keep turning back
			wanted_heading = Math.PI / 2 + heading_difference_from_start;
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

		if (Math.abs(leftForward) < power_epsilon) leftForward = 0.0;
		if (Math.abs(rightForward) < power_epsilon) rightForward = 0.0;
		if (Math.abs(frontSideways) < power_epsilon) frontSideways = 0.0;
		if (Math.abs(backSideways) < power_epsilon) backSideways = 0.0;

		hardware.leftForwardMotor.setPower(leftForward);
		hardware.rightForwardMotor.setPower(rightForward);
		hardware.frontSidewaysMotor.setPower(frontSideways);
		hardware.backSidewaysMotor.setPower(backSideways);

		// Disable breaking if we don't need it
		boolean not_moving =
			leftForward == 0 &&
			rightForward == 0 &&
			frontSideways == 0 &&
			backSideways == 0;

		if (not_moving) {
			if (stopped_moving_time == 0) {
				stopped_moving_time = System.currentTimeMillis();
			}
		} else {
			stopped_moving_time = 0;
		}

		if (not_moving && System.currentTimeMillis() - stopped_moving_time > stop_breaking_after_ms) {
			hardware.leftForwardMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
			hardware.rightForwardMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
			hardware.frontSidewaysMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
			hardware.backSidewaysMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		} else {
			hardware.leftForwardMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			hardware.rightForwardMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			hardware.frontSidewaysMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			hardware.backSidewaysMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		}

		if (isIMUOk(current_orientation)) {
			last_robot_orientation = current_orientation;
		} else {
			tryFixIMU();
		}

		if (debug) {

			callingOpMode.telemetry.addLine("-- Drivetrain --");
			callingOpMode.telemetry.addData("field centric rotation", fieldCentricRotation);
			callingOpMode.telemetry.addData("stoppped_time         ", stopped_moving_time);
			callingOpMode.telemetry.addData("breaking?             ", hardware.frontSidewaysMotor.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE);
			callingOpMode.telemetry.addData("translation power     ", translation_power);
			callingOpMode.telemetry.addData("translation local  dir", Math.toDegrees(translation_direction));

			if (fieldCentricTranslation) {
				callingOpMode.telemetry.addData("translation global dir",  Math.toDegrees(translation_inputs.second));
			}

			callingOpMode.telemetry.addData("Left  Forward motor power", leftForward);
			callingOpMode.telemetry.addData("Right Forward motor power", rightForward);
			callingOpMode.telemetry.addData("Front Sideways motor power", frontSideways);
			callingOpMode.telemetry.addData("Back  Sideways motor power", backSideways);

			callingOpMode.telemetry.addData("heading difference", Math.toDegrees(heading_difference_from_start));
			callingOpMode.telemetry.addData("Angular velocity (rad / s)", angular_velocity_rad_per_s);
			callingOpMode.telemetry.addData("IMU ok", isIMUOk(current_orientation));
			callingOpMode.telemetry.addData("Last IMU fix", last_imu_reset_time);

			callingOpMode.telemetry.addData("Rotating due to manual time", started_rotating_due_to_manual_stick_time);
		}
	}
}
