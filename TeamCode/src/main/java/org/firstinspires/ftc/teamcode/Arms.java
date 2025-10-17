package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/// Our arms logic.
///
/// Two continuous servos raise and lower the arms
///
/// Two normal range servos open and close them
///
/// Note: The servo range has to be set manually with a programmator!
public class Arms {
	LinearOpMode callingOpMode;
	Hardware hardware;

	public double vertical_power_multiplier = 1.0;
	public double horizontal_power_multiplier = 1.0;

	public double open_closed_servo_speed_positions_per_second = 0.75;

	/// Used for manually calculating the servo's position delta from a given speed
	public long last_speed_calculation_time = 0;

	public Arms(LinearOpMode opMode, Hardware hw_map) {
		callingOpMode = opMode;
		hardware = hw_map;

		hardware.armOpenClosedServoLeft.setPosition(0);
		hardware.armOpenClosedServoRight.setPosition(0);
	}

	/// Computes a servo speed from time delta from the previous call
	public double open_closed_speed() {
		long now = System.currentTimeMillis();

		if (last_speed_calculation_time == 0) {
			last_speed_calculation_time = now;
			return 0.0;
		}

		double elapsed_s = (double)(now - last_speed_calculation_time) / 1000.0;

		last_speed_calculation_time = now;

		return open_closed_servo_speed_positions_per_second * elapsed_s;
	}

	// Positive power -> up
	// Negative power -> down
	public void update_up_down(Double left_servo_power, Double right_servo_power) {

		left_servo_power = left_servo_power * vertical_power_multiplier;
		left_servo_power = Double.max(-1.0, left_servo_power);
		left_servo_power = Double.min(1.0, left_servo_power);

		right_servo_power = right_servo_power * vertical_power_multiplier;
		right_servo_power = Double.max(-1.0, right_servo_power);
		right_servo_power = Double.min(1.0, right_servo_power);

		hardware.armHeightServoLeft.setPower(left_servo_power);
		hardware.armHeightServoRight.setPower(right_servo_power);
	}

	// Positive power -> up
	// Negative power -> down
	public void update_up_down(Double servo_power) {

		servo_power = servo_power * vertical_power_multiplier;
		servo_power = Double.max(-1.0, servo_power);
		servo_power = Double.min(1.0, servo_power);

		hardware.armHeightServoLeft.setPower(servo_power);
		hardware.armHeightServoRight.setPower(servo_power);
	}

	// Positive power -> open
	// Negative power -> close
	public void update_open_closed(Double left_servo_power, Double right_servo_power) {

		left_servo_power = left_servo_power * horizontal_power_multiplier;
		left_servo_power = Double.max(-1.0, left_servo_power);
		left_servo_power = Double.min(1.0, left_servo_power);

		right_servo_power = right_servo_power * horizontal_power_multiplier;
		right_servo_power = Double.max(-1.0, right_servo_power);
		right_servo_power = Double.min(1.0, right_servo_power);

		double speed = open_closed_speed();

		hardware.armOpenClosedServoLeft.setPosition(hardware.armOpenClosedServoLeft.getPosition() + left_servo_power * speed);
		hardware.armOpenClosedServoRight.setPosition(hardware.armOpenClosedServoRight.getPosition() + right_servo_power * speed);
	}

	// Positive power -> open
	// Negative power -> close
	public void update_open_closed(Double servo_power) {

		servo_power = servo_power * horizontal_power_multiplier;
		servo_power = Double.max(-1.0, servo_power);
		servo_power = Double.min(1.0, servo_power);

		double speed = open_closed_speed();

		hardware.armOpenClosedServoLeft.setPosition(hardware.armOpenClosedServoLeft.getPosition() + servo_power * speed);
		hardware.armOpenClosedServoRight.setPosition(hardware.armOpenClosedServoRight.getPosition() + servo_power * speed);
	}
}
