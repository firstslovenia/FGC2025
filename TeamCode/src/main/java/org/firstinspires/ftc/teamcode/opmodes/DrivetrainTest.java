package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Vector2D;

@TeleOp(name = "DrivetrainTest")
public class DrivetrainTest extends LinearOpMode {

	Hardware hardware;
	Drivetrain drivetrain;

	@Override
	public void runOpMode() {

		hardware = new Hardware(this);
		hardware.init();

		drivetrain = new Drivetrain(this, hardware);
		drivetrain.fieldCentricTranslation = true;
		drivetrain.fieldCentricRotation = true;
		drivetrain.resetStartingDirection();

		waitForStart();

		while (opModeIsActive()) {
			drivetrain.update(new Vector2D(gamepad1.left_stick_x, gamepad1.left_stick_y), new Vector2D(gamepad1.right_stick_x, gamepad1.right_stick_y));
			telemetry.update();
		}
	}
}
