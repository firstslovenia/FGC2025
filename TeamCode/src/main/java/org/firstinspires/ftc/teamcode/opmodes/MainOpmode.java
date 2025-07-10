package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Lifter;
import org.firstinspires.ftc.teamcode.generic.Vector2D;

@TeleOp(name = "Main")
public class MainOpmode extends LinearOpMode {

	Hardware hardware;
	Drivetrain drivetrain;
	Lifter lifter;

	@Override
	public void runOpMode() {

		hardware = new Hardware(this);
		hardware.init();

		drivetrain = new Drivetrain(this, hardware);
		drivetrain.fieldCentricTranslation = true;
		drivetrain.fieldCentricRotation = true;
		drivetrain.resetStartingDirection();

		lifter = new Lifter(this, hardware);

		waitForStart();

		while (opModeIsActive()) {

			lifter.update((double) gamepad2.right_stick_y);

			if (gamepad1.guide) {
				drivetrain.resetStartingDirection();
			}

			if (gamepad1.back) {
				drivetrain.fieldCentricRotation = true;
				drivetrain.fieldCentricTranslation = true;
			}

			if (gamepad1.start) {
				drivetrain.fieldCentricRotation = false;
				drivetrain.fieldCentricTranslation = false;
			}

			drivetrain.update(new Vector2D(gamepad1.left_stick_x, gamepad1.left_stick_y), new Vector2D(gamepad1.right_stick_x, gamepad1.right_stick_y));
			telemetry.update();
		}
	}
}
