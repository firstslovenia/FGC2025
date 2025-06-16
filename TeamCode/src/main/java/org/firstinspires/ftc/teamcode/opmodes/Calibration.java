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

	@Override
	public void runOpMode() {

		hardware = new Hardware(this);
		hardware.init();

		odometry = new Odometry(this, hardware);

		drivetrain = new Drivetrain(this, hardware, odometry);
		drivetrain.rotationMultiplier = 0.25;
		drivetrain.fieldCentricRotation = false;
		drivetrain.fieldCentricTranslation = false;

		waitForStart();

		while (opModeIsActive()) {

			int frontSideways = hardware.frontSidewaysMotor.getCurrentPosition();
			int backSideways = hardware.backSidewaysMotor.getCurrentPosition();
			int rightForward = hardware.rightForwardMotor.getCurrentPosition();
			int leftForward = hardware.leftForwardMotor.getCurrentPosition();

			Double rotation_calibration_magic = (double) (leftForward + frontSideways - rightForward - backSideways);
			Double forward_translation_calibration_magic = (double) (leftForward + rightForward);
			Double side_translation_calibration_magic = (double) (frontSideways + backSideways);

			telemetry.addLine("-- Calibration --");
			telemetry.addData("rotation magic", rotation_calibration_magic);
			telemetry.addData("forward translation magic", forward_translation_calibration_magic);
			telemetry.addData("sideways translation magic", side_translation_calibration_magic);

			drivetrain.update(new Vector2D(gamepad1.left_stick_x, gamepad1.left_stick_y), new Vector2D(gamepad1.right_stick_x, gamepad1.right_stick_y));
			odometry.update();
			telemetry.update();
		}
	}
}
