package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

	/// Did by vibes
	///
	/// Source: I made it up
	public double open_closed_motor_speed_positions_per_second = -200.0;

	public int min_left_open_closed_motor_position = -135;
	public int max_left_open_closed_motor_position = -45;
	public int min_right_open_closed_motor_position = -135;
	public int max_right_open_closed_motor_position = -45;

	/// Used for manually calculating the servo's position delta from a given speed
	public long last_speed_calculation_time = 0;

	public Arms(LinearOpMode opMode, Hardware hw_map) {
		callingOpMode = opMode;
		hardware = hw_map;

		hardware.armOpenClosedMotorLeft.setTargetPosition(0);
		hardware.armOpenClosedMotorRight.setTargetPosition(0);

		hardware.armOpenClosedMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		hardware.armOpenClosedMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		hardware.armOpenClosedMotorLeft.setPower(0.5);
		hardware.armOpenClosedMotorRight.setPower(0.5);
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

		return open_closed_motor_speed_positions_per_second * elapsed_s;
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
		update_up_down(servo_power, servo_power);
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

		int left_pos = (int) (hardware.armOpenClosedMotorLeft.getTargetPosition() + left_servo_power * speed);
		int right_pos = (int) (hardware.armOpenClosedMotorRight.getTargetPosition() + right_servo_power * speed);

		//left_pos = Math.min(left_pos, max_left_open_closed_motor_position);
		//right_pos = Math.min(right_pos, max_right_open_closed_motor_position);

		//left_pos = Math.max(left_pos, min_left_open_closed_motor_position);
		//right_pos = Math.max(right_pos, min_right_open_closed_motor_position);

		hardware.armOpenClosedMotorLeft.setTargetPosition(left_pos);
		hardware.armOpenClosedMotorRight.setTargetPosition(right_pos);
	}

	// Positive power -> open
	// Negative power -> close
	public void update_open_closed(Double servo_power) {
		update_open_closed(servo_power, servo_power);
	}
}
