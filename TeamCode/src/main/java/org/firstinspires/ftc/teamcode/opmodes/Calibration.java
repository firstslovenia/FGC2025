package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Vector2D;

/// Helps calibrate odometry (for field centric control)
///
/// To calibrate rotation, mount a compass to the robot and turn it several multiples of 2 * Pi (full circles)
///
/// then divide
/// rotation calibration magic / number of turns
/// and set magicStepsForTwoPi to that number in Odometry.java
///
@TeleOp(name = "Calibration")
public class Calibration extends LinearOpMode {

	Hardware hardware;
	Drivetrain drivetrain;
	Odometry odometry;

	/// Number of turns done, just as an aid
	int n_turns_done = 0;

	/// The odometry heading offset from the last loop, to check if we've completed a full circle
	double last_odometry_heading = 0.0;

	/// Whether bumpers were pressed in the last loop (to handle button presses properly)
	boolean last_left_bumper = false;
	boolean last_right_bumper = false;

	@Override
	public void runOpMode() {

		hardware = new Hardware(this);
		hardware.init();

		odometry = new Odometry(this, hardware);

		drivetrain = new Drivetrain(this, hardware);
		drivetrain.rotationMultiplier = 0.25;
		drivetrain.fieldCentricRotation = false;
		drivetrain.fieldCentricTranslation = false;

		waitForStart();

		while (opModeIsActive()) {

			if (gamepad1.left_bumper && !last_left_bumper) {
				drivetrain.rotationMultiplier /= 2.0;
			}

			if (gamepad1.right_bumper && !last_right_bumper) {
				drivetrain.rotationMultiplier *= 2.0;
			}

			int frontSideways = hardware.frontSidewaysMotor.getCurrentPosition();
			int backSideways = hardware.backSidewaysMotor.getCurrentPosition();
			int rightForward = hardware.rightForwardMotor.getCurrentPosition();
			int leftForward = hardware.leftForwardMotor.getCurrentPosition();

			Double rotation_calibration_magic = (double) (leftForward + frontSideways - rightForward - backSideways);
			Double forward_translation_calibration_magic = (double) (leftForward + rightForward);
			Double side_translation_calibration_magic = (double) (frontSideways + backSideways);

			telemetry.addLine("-- Calibration --");
			telemetry.addData("rotation magic", rotation_calibration_magic);
			telemetry.addData("turns done    ", n_turns_done);
			telemetry.addData("forward translation magic", forward_translation_calibration_magic);
			telemetry.addData("sideways translation magic", side_translation_calibration_magic);

			drivetrain.update(new Vector2D(gamepad1.left_stick_x, gamepad1.left_stick_y), new Vector2D(gamepad1.right_stick_x, gamepad1.right_stick_y));
			odometry.update();
			telemetry.update();

			// If we had a sudden jump, we probably jumped from (lots of rotation) to (no rotation), meaning we completed a turn
			if (Math.abs(odometry.heading - last_odometry_heading) > Math.PI) {
				n_turns_done += 1;
			}

			last_odometry_heading = odometry.heading;
			last_left_bumper = gamepad1.left_bumper;
			last_right_bumper = gamepad1.right_bumper;
		}
	}
}
