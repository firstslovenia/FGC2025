package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Vector2D;

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
		drivetrain.rotationMultiplier = 0.5;
		drivetrain.fieldCentricRotation = false;
		drivetrain.fieldCentricTranslation = false;

		waitForStart();

		while (opModeIsActive()) {
			drivetrain.update(new Vector2D(gamepad1.left_stick_x, gamepad1.left_stick_y), new Vector2D(gamepad1.right_stick_x, gamepad1.right_stick_y));
			odometry.update();
			telemetry.update();
		}
	}
}
