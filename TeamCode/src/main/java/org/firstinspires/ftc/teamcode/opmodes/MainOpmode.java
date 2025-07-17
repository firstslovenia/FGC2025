package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Arms;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Lifter;
import org.firstinspires.ftc.teamcode.generic.Vector2D;

@TeleOp(name = "Main")
public class MainOpmode extends LinearOpMode {

	Hardware hardware;
	Drivetrain drivetrain;
	Lifter lifter;
	Arms arms;

	@Override
	public void runOpMode() {

		hardware = new Hardware(this);
		hardware.init();

		drivetrain = new Drivetrain(this, hardware);
		drivetrain.fieldCentricTranslation = true;
		drivetrain.fieldCentricRotation = true;
		drivetrain.resetStartingDirection();

		lifter = new Lifter(this, hardware);

		arms = new Arms(this, hardware);

		waitForStart();

		while (opModeIsActive()) {

			double lifter_power = gamepad1.right_trigger + -gamepad1.left_trigger;

			if (gamepad1.left_bumper) {
				lifter_power -= 0.5;
			}

			if (gamepad1.right_bumper) {
				lifter_power += 0.5;
			}

			lifter.update(lifter_power);

			double arms_vertical_power = 0.0;

			if (gamepad2.a) {
				arms_vertical_power -= 1.0;
			}

			if (gamepad2.b) {
				arms_vertical_power += 1.0;
			}

			arms.update_up_down(arms_vertical_power);

			double arms_horizontal_power = 0.0;

			if (gamepad2.x) {
				arms_horizontal_power -= 1.0;
			}

			if (gamepad2.y) {
				arms_horizontal_power += 1.0;
			}

			arms.update_open_closed(arms_horizontal_power);

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

			if (gamepad1.x) {
				drivetrain.keepHeading = true;
			}

			if (gamepad1.y) {
				drivetrain.keepHeading = false;
			}

			Vector2D translation_vector = new Vector2D(gamepad1.left_stick_x, gamepad1.left_stick_y);

			if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
				translation_vector = new Vector2D(0.0, 0.0);

				if (gamepad1.dpad_up) { translation_vector.y += 0.65; }
				if (gamepad1.dpad_down) { translation_vector.y -= 0.65; }
				if (gamepad1.dpad_right) { translation_vector.x += 0.65; }
				if (gamepad1.dpad_left) { translation_vector.x -= 0.65; }
			}

			drivetrain.update(translation_vector, new Vector2D(gamepad1.right_stick_x, gamepad1.right_stick_y));
			telemetry.update();
		}
	}
}
