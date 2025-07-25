package org.firstinspires.ftc.teamcode.generic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Optional;

/// Generic PIDF controller
public abstract class PIDController {

	/// Optional prefix in the debug data
	public String telemetry_prefix = "";

	/// Optional calling opmode which can be set to print telemetry
	public Optional<LinearOpMode> callingOpMode;

	/// Whether or not to print debug values to telemetry
	public boolean debug = true;

	public PIDController() {
		this.callingOpMode = Optional.empty();
		debug = false;
	}

	public PIDController(Optional<LinearOpMode> callingOpmode, String telemetry_prefix) {
		this.callingOpMode = callingOpmode;
		this.telemetry_prefix = telemetry_prefix;
		debug = true;
	}

	/// Our proportional (k_p * e) coefficient
	public abstract double get_coefficient_p();

	/// Our integral (k_i * integral e dt) coefficient
	public abstract double get_coefficient_i();

	/// Our derivative (k_d * de/dt) coefficient
	public abstract double get_coefficient_d();

	/// Our feed-forward (+ k_f) coefficient
	public abstract double get_coefficient_f();

	public double epsilon_integral = 0.0;

	public double previous_epsilon = 0.0;

	public long previous_epsilon_time = 0;

	/// Resets our control loop (integral and derivate) values
	public void reset() {
		epsilon_integral = 0.0;
		previous_epsilon = 0.0;
		previous_epsilon_time = 0;
	}

	/// Adds one e * dt to our saved integral value
	public void update_integral() {
		if (previous_epsilon_time == 0) {
			return;
		}

		double delta_time = (System.currentTimeMillis() - previous_epsilon_time) / 1000.0;
		epsilon_integral += delta_time * epsilon();
	}

	/// Calculates d epsilon / dt, if we have the previous value
	public Optional<Double> calculate_derivative() {
		if (previous_epsilon_time == 0) {
			return Optional.empty();
		}

		double delta_time = (System.currentTimeMillis() - previous_epsilon_time) / 1000.0;
		double delta_epsilon = epsilon() - previous_epsilon;

		return Optional.of(delta_epsilon / delta_time);
	}

	/// Updates the previous epsilon values, should be the last function called in update()
	public void update_set_previous() {
		previous_epsilon = epsilon();
		previous_epsilon_time = System.currentTimeMillis();
	}

	/// Runs the update loop, calling output with the calculated control value
	public void update() {
		update_integral();

		double p = get_coefficient_p() * epsilon();
		double i = get_coefficient_i() * epsilon_integral;
		double d = get_coefficient_d() * calculate_derivative().orElse(0.0);
		double f = get_coefficient_f();

		// Compensate for derivate jumping randomly (2 PI -> 0)
		if (Math.abs(d) > 1.0) {
			d = 0.0;
		}

		// Make feed-forward go the same way as proportional
		if (p < 0.0) {
			f = -f;
		}

		if (callingOpMode.isPresent() && debug) {

			LinearOpMode opmode = callingOpMode.get();

			opmode.telemetry.addData(telemetry_prefix + "proportional", p);
			opmode.telemetry.addData(telemetry_prefix + "integral", i);
			opmode.telemetry.addData(telemetry_prefix + "derivative", d);
			opmode.telemetry.addData(telemetry_prefix + "constant", f);
		}

		double value = p + i + d + f;

		output(value);
		update_set_previous();
	}

	/// Our output control function, should e.g. set a motor's power
	public abstract void output(double value);

	/// Our function which tells us how far we are from the target value
	public abstract double epsilon();
}
