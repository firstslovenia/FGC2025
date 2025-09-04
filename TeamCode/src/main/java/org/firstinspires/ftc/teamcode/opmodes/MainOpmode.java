package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Arms;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Lifter;
import org.firstinspires.ftc.teamcode.State;
import org.firstinspires.ftc.teamcode.generic.Vector2D;

@TeleOp(name = "Main")
public class MainOpmode extends LinearOpMode {

	Hardware hardware;
	Drivetrain drivetrain;
	Lifter lifter;
	Arms arms;

	State state = State.Drive;

	// Keeps track of when the next and previous state buttons were pressed, for debouncing
	boolean next_state_button_last = false;

	long next_state_button_pressed_time = 0;
	boolean previous_state_button_last = false;
	long previous_state_button_pressed_time = 0;

	/// The minimum interval of pressed that we consider not being a bounce
	long button_bounce_ms = 200;

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

			double lifter_power = gamepad2.right_trigger + -gamepad2.left_trigger;

			if (gamepad2.left_bumper) {
				lifter_power -= 0.5;
			}

			if (gamepad2.right_bumper) {
				lifter_power += 0.5;
			}

			if (state == State.CatchRope || state == State.Climb) {
				if (!lifter.ready) {
					lifter.initialize();
				}

				lifter.update(lifter_power);
			} else {
				if (lifter.ready) {
					lifter.uninitialize();
				}
			}

			double arms_vertical_power = 0.0;

			if (gamepad2.a) {
				arms_vertical_power -= 1.0;
			}

			if (gamepad2.y) {
				arms_vertical_power += 1.0;
			}

			arms.update_up_down(arms_vertical_power);
			if (gamepad2.dpad_left) {
				arms.update_up_down(arms_vertical_power, 0.0);
			} else if (gamepad2.dpad_right) {
				arms.update_up_down(0.0, arms_vertical_power);
			} else {
				arms.update_up_down(arms_vertical_power);
			}

			double arms_horizontal_power = 0.0;

			if (gamepad2.b) {
				arms_horizontal_power += 1.0;
			}

			if (gamepad2.x) {
				arms_horizontal_power -= 1.0;
			}

			if (gamepad2.dpad_left) {
				arms.update_open_closed(arms_horizontal_power * 0.25, 0.0);
			} else if (gamepad2.dpad_right) {
				arms.update_open_closed(0.0, arms_horizontal_power * 0.25);
			} else {
				arms.update_open_closed(arms_horizontal_power);
			}

			if (gamepad1.guide) {
				drivetrain.resetStartingDirection();
			}

			if (gamepad1.x) {
				drivetrain.fieldCentricRotation = true;
				drivetrain.fieldCentricTranslation = true;
			}

			if (gamepad1.y) {
				drivetrain.fieldCentricRotation = false;
				drivetrain.fieldCentricTranslation = false;
			}

			long now = System.currentTimeMillis();
			long since_next_state_button_pressed_ms = now - next_state_button_pressed_time;
			long since_previous_state_button_pressed_ms = now - previous_state_button_pressed_time;

			if (gamepad1.a && !next_state_button_last && since_next_state_button_pressed_ms > button_bounce_ms) {
				next_state_button_pressed_time = now;

				switch (state) {
					case Drive:
						state = State.CatchRope;
						break;

					case CatchRope:
						state = State.Climb;
						break;

					default:
						break;
				}
			}

			if (gamepad1.b && !previous_state_button_last && since_previous_state_button_pressed_ms > button_bounce_ms) {
				previous_state_button_pressed_time = now;

				switch (state) {
					case Climb:
						state = State.CatchRope;
						break;

					case CatchRope:
						state = State.Drive;
						break;

					default:
						break;
				}
			}

			next_state_button_last = gamepad1.a;
			previous_state_button_last = gamepad1.b;

			Vector2D translation_vector = new Vector2D(gamepad1.left_stick_x, gamepad1.left_stick_y);

			if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
				translation_vector = new Vector2D(0.0, 0.0);

				if (gamepad1.dpad_up) { translation_vector.y += 0.65; }
				if (gamepad1.dpad_down) { translation_vector.y -= 0.65; }
				if (gamepad1.dpad_right) { translation_vector.x += 0.65; }
				if (gamepad1.dpad_left) { translation_vector.x -= 0.65; }
			}

			if (state == State.Drive || state == State.CatchRope) {
				drivetrain.update(translation_vector, new Vector2D(gamepad1.right_stick_x, gamepad1.right_stick_y));
			}

			telemetry.addData("state", state.toString());
			telemetry.update();
		}
	}
}
