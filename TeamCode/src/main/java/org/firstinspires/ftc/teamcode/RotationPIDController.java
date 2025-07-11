package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.generic.PIDController;

import java.util.Optional;

/// A PID controller for our drivetrain's field-centric rotation
public class RotationPIDController extends PIDController {

	RotationPIDController(LinearOpMode opMode) {
		this.callingOpMode = Optional.of(opMode);
		this.telemetry_prefix = "";
		this.debug = true;
	}

	/// Our proportional (k_p * e) coefficient
	public double get_coefficient_p() {
		return 0.8;
	}

	/// Our integral (k_i * integral e dt) coefficient
	public double get_coefficient_i() {
		return 0.0;
	}

	/// Our derivative (k_d * de/dt) coefficient
	public double get_coefficient_d() {
		return 0.0;
	}

	/// Our feed-forward (+ k_f) coefficient
	public double get_coefficient_f() {
		return 0.0;
	}

	/// Our manually set epsilon value in radians - how far we are from our wanted rotation
	///
	/// a negative value denotes the other way, and should result in a negative output
	public double needed_turn = 0.0;

	/// The last loop's output control value
	public double power_output = 0.0;

	/// Our output control function, should e.g. set a motor's power
	public void output(double value) {
		power_output = value;
	}

	/// Our function which tells us how far we are from the target value
	public double epsilon() {
		return needed_turn;
	}
}
