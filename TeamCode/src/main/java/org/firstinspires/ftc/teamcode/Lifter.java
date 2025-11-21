package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/// Our lifter logic.
///
/// A motor turns some gears which in-turn slowly rotate a thingy which catches and spools the rope.
public class Lifter {
	LinearOpMode callingOpMode;
	Hardware hardware;

	/// When we started the final ascent -> we can still automagically climb for 4 seconds
	long started_final_ascent_time = 0;

	/// How long the final ascent lasts
	long final_ascent_duration_ms = 4000;

	public Lifter(LinearOpMode opMode, Hardware hw_map) {
		callingOpMode = opMode;
		hardware = hw_map;
	}

	/// Called to enable the motors, setup what we need to be able to use the lifter
	public void initialize() {
		hardware.lifterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		hardware.lifterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	/// Called to undo initialize
	public void uninitialize() {
		hardware.lifterMotorLeft.setPower(0);
		hardware.lifterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		hardware.lifterMotorRight.setPower(0);
		hardware.lifterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
	}

	public void update(Double climb_up_power) {

		boolean is_in_final_ascent = started_final_ascent_time != 0;
		if (is_in_final_ascent) {

			boolean final_ascent_over = (System.currentTimeMillis() - started_final_ascent_time) >= final_ascent_duration_ms;

			if (final_ascent_over) {
				started_final_ascent_time = 0;
				hardware.lifterMotorLeft.setPower(0);
				hardware.lifterMotorRight.setPower(0);
			}

			return;
		}

		climb_up_power = Double.max(-1.0, climb_up_power);
		climb_up_power = Double.min(1.0, climb_up_power);

		hardware.lifterMotorLeft.setPower(climb_up_power);
		hardware.lifterMotorRight.setPower(climb_up_power);
	}

	/// Starts the final climb -> no more input from the controller, automatically stops after 4 seconds
	public void start_final_ascent() {
		started_final_ascent_time = System.currentTimeMillis();
	}
}
