package org.firstinspires.ftc.teamcode.generic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Optional;

/// A generic PID controller controlled with setting and reading fields
public class GenericPIDController extends PIDController {

	public GenericPIDController(LinearOpMode opMode, double p, double i, double d, double f) {
		this.callingOpMode = Optional.of(opMode);
		this.telemetry_prefix = "";
		this.debug = true;
		this.p_coefficient = p;
		this.i_coefficient = i;
		this.d_coefficient = d;
		this.f_coefficient = f;
	}

	public GenericPIDController(LinearOpMode opMode, String telemetry_prefix, double p, double i, double d, double f) {
		this.callingOpMode = Optional.of(opMode);
		this.telemetry_prefix = telemetry_prefix;
		this.debug = true;
		this.p_coefficient = p;
		this.i_coefficient = i;
		this.d_coefficient = d;
		this.f_coefficient = f;
	}

	double p_coefficient = 0.0;
	double i_coefficient = 0.0;
	double d_coefficient = 0.0;
	double f_coefficient = 0.0;

	/// Our proportional (k_p * e) coefficient
	public double get_coefficient_p() {
		return p_coefficient;
	}

	/// Our integral (k_i * integral e dt) coefficient
	public double get_coefficient_i() {
		return i_coefficient;
	}

	/// Our derivative (k_d * de/dt) coefficient
	public double get_coefficient_d() {
		return d_coefficient;
	}

	/// Our feed-forward (+ k_f) coefficient
	public double get_coefficient_f() {
		return f_coefficient;
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
