package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.generic.PIDController;

import java.util.Optional;

/// A generic PID controller controlled with setting and reading fields
public class GenericPIDController extends PIDController {

	GenericPIDController(LinearOpMode opMode) {
		this.callingOpMode = Optional.of(opMode);
		this.telemetry_prefix = "";
		this.debug = true;
	}

	GenericPIDController(LinearOpMode opMode, String telemetry_prefix) {
		this.callingOpMode = Optional.of(opMode);
		this.telemetry_prefix = telemetry_prefix;
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

	/// Our manually set epsilon / error value
	public double error = 0.0;

	/// The last loop's output control value
	public double output = 0.0;

	/// Our output control function, should e.g. set a motor's power
	public void output(double value) {
		output = value;
	}

	/// Our function which tells us how far we are from the target value
	public double epsilon() {
		return error;
	}
}
