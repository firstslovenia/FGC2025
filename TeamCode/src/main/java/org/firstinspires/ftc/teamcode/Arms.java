package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/// Our arms logic.
///
/// A continous servo lowers and raises the arms for now
public class Arms {
	LinearOpMode callingOpMode;
	Hardware hardware;

	public double vertical_power_multiplier = 1.0;
	public double horizontal_power_multiplier = 1.0;

	public Arms(LinearOpMode opMode, Hardware hw_map) {
		callingOpMode = opMode;
		hardware = hw_map;
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

		hardware.armOpenClosedServoLeft.setPower(left_servo_power);
		hardware.armOpenClosedServoRight.setPower(right_servo_power);
	}

	// Positive power -> open
	// Negative power -> close
	public void update_open_closed(Double servo_power) {

		servo_power = servo_power * horizontal_power_multiplier;
		servo_power = Double.max(-1.0, servo_power);
		servo_power = Double.min(1.0, servo_power);

		hardware.armOpenClosedServoLeft.setPower(servo_power);
		hardware.armOpenClosedServoRight.setPower(servo_power);
	}
}
