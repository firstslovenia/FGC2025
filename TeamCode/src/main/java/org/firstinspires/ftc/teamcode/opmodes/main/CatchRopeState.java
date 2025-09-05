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

		Vector2D translation_vector = new Vector2D(opmode.gamepad1.left_stick_x, opmode.gamepad1.left_stick_y);

		if (opmode.gamepad1.dpad_up || opmode.gamepad1.dpad_down || opmode.gamepad1.dpad_left || opmode.gamepad1.dpad_right) {

			translation_vector = new Vector2D(0.0, 0.0);

			if (opmode.gamepad1.dpad_up) { translation_vector.y += 0.65; }
			if (opmode.gamepad1.dpad_down) { translation_vector.y -= 0.65; }
			if (opmode.gamepad1.dpad_right) { translation_vector.x += 0.65; }
			if (opmode.gamepad1.dpad_left) { translation_vector.x -= 0.65; }
		}

		drivetrain.update(translation_vector, new Vector2D(opmode.gamepad1.right_stick_x, opmode.gamepad1.right_stick_y));
	}
}
