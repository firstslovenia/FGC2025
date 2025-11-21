package org.firstinspires.ftc.teamcode.opmodes.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Arms;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Lifter;
import org.firstinspires.ftc.teamcode.generic.State;

public class ClimbState implements State {
	Hardware hardware;
	Arms arms;
	Lifter lifter;

	LinearOpMode opmode;

	public ClimbState(LinearOpMode opmode, Hardware hardware, Arms arms, Lifter lifter) {
		this.opmode = opmode;
		this.hardware = hardware;
		this.arms = arms;
		this.lifter = lifter;
	}

	@Override
	public void on_switch_from(StateEnum old_state) {
		if (old_state != StateEnum.CatchRope) {
			lifter.initialize();
		}
	}

	@Override
	public void on_switch_to(StateEnum new_state) {
		switch (new_state) {
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

		if (opmode.gamepad2.guide) {
			lifter.start_final_ascent();
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
			arms.update_open_closed(arms_horizontal_power * 0.5, 0.0);
		} else if (opmode.gamepad2.dpad_right) {
			arms.update_open_closed(0.0, arms_horizontal_power * 0.5);
		} else {
			arms.update_open_closed(arms_horizontal_power);
		}
	}
}

