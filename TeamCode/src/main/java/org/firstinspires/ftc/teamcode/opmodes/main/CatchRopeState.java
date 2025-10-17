package org.firstinspires.ftc.teamcode.opmodes.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Arms;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Lifter;
import org.firstinspires.ftc.teamcode.generic.State;
import org.firstinspires.ftc.teamcode.generic.Vector2D;

public class CatchRopeState implements State {
	Hardware hardware;
	Drivetrain drivetrain;
	Arms arms;
	Lifter lifter;

	LinearOpMode opmode;

	boolean toggle_field_centric_button_last = false;
	long toggle_field_centric_button_pressed_time = 0;

	boolean toggle_keep_heading_button_last = false;
	long toggle_keep_heading_button_pressed_time = 0;

	/// The minimum interval of pressed that we consider not being a bounce
	static long button_bounce_ms = 200;

	public CatchRopeState(LinearOpMode opmode, Hardware hardware, Drivetrain drivetrain, Arms arms, Lifter lifter) {
		this.opmode = opmode;
		this.hardware = hardware;
		this.drivetrain = drivetrain;
		this.arms = arms;
		this.lifter = lifter;
	}

	@Override
	public void on_switch_from(StateEnum old_state) {
		if (old_state != StateEnum.Climb) {
			lifter.initialize();
		}
	}

	@Override
	public void on_switch_to(StateEnum new_state) {
		switch (new_state) {
			case Climb:
				drivetrain.disable();
				break;

			case Drive:
				lifter.uninitialize();
				break;

			default:
				break;
		}
	}

	@Override
	public void loop() {

		double lifter_power = opmode.gamepad2.right_trigger + -opmode.gamepad2.left_trigger;

		if (opmode.gamepad2.left_bumper) {
			lifter_power -= 0.5;
		}

		if (opmode.gamepad2.right_bumper) {
			lifter_power += 0.5;
		}

		lifter.update(lifter_power);

		double arms_vertical_power = 0.0;

		if (opmode.gamepad2.a) {
			arms_vertical_power -= 1.0;
		}

		if (opmode.gamepad2.y) {
			arms_vertical_power += 1.0;
		}

		if (opmode.gamepad2.dpad_left) {
			arms.update_up_down(arms_vertical_power, 0.0);
		} else if (opmode.gamepad2.dpad_right) {
			arms.update_up_down(0.0, arms_vertical_power);
		} else {
			arms.update_up_down(arms_vertical_power);
		}

		double arms_horizontal_power = 0.0;

		if (opmode.gamepad2.b) {
			arms_horizontal_power += 1.0;
		}

		if (opmode.gamepad2.x) {
			arms_horizontal_power -= 1.0;
		}

		if (opmode.gamepad2.dpad_left) {
			arms.update_open_closed(arms_horizontal_power * 0.25, 0.0);
		} else if (opmode.gamepad2.dpad_right) {
			arms.update_open_closed(0.0, arms_horizontal_power * 0.25);
		} else {
			arms.update_open_closed(arms_horizontal_power);
		}

		if (opmode.gamepad1.guide) {
			drivetrain.resetStartingDirection();
		}

		if (opmode.gamepad1.a) {
			drivetrain.fieldCentricRotation = true;
			drivetrain.fieldCentricTranslation = true;
		}

		if (opmode.gamepad1.b) {
			drivetrain.fieldCentricRotation = false;
			drivetrain.fieldCentricTranslation = false;
		}

		long now = System.currentTimeMillis();
		long since_field_centric_toggle_button_pressed_ms = now - toggle_field_centric_button_pressed_time;

		if (opmode.gamepad1.a && !toggle_field_centric_button_last && since_field_centric_toggle_button_pressed_ms > button_bounce_ms ) {
			drivetrain.fieldCentricRotation = !drivetrain.fieldCentricRotation;
			drivetrain.fieldCentricTranslation = !drivetrain.fieldCentricTranslation;

			toggle_field_centric_button_pressed_time = now;
		}
		toggle_field_centric_button_last = opmode.gamepad1.a;

		long since_keep_heading_toggle_button_pressed_ms = now - toggle_keep_heading_button_pressed_time;

		if (opmode.gamepad1.b && !toggle_keep_heading_button_last && since_keep_heading_toggle_button_pressed_ms > button_bounce_ms ) {
			drivetrain.keepHeading = !drivetrain.keepHeading;

			toggle_keep_heading_button_pressed_time = now;
		}
		toggle_keep_heading_button_last = opmode.gamepad1.b;

		Vector2D translation_vector = new Vector2D(opmode.gamepad1.left_stick_x, opmode.gamepad1.left_stick_y);
		boolean override_field_centric = false;

		if (opmode.gamepad1.dpad_up || opmode.gamepad1.dpad_down || opmode.gamepad1.dpad_left || opmode.gamepad1.dpad_right) {

			translation_vector = new Vector2D(0.0, 0.0);

			if (opmode.gamepad1.dpad_up) { translation_vector.y += 0.65; }
			if (opmode.gamepad1.dpad_down) { translation_vector.y -= 0.65; }
			if (opmode.gamepad1.dpad_right) { translation_vector.x += 0.65; }
			if (opmode.gamepad1.dpad_left) { translation_vector.x -= 0.65; }

			override_field_centric = true;
		}

		drivetrain.update(translation_vector, new Vector2D(opmode.gamepad1.right_stick_x, opmode.gamepad1.right_stick_y), override_field_centric);
	}
}
