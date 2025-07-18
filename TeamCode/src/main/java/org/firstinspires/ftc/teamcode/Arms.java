package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/// Our arms logic.
///
/// A continous servo lowers and raises the arms for now
public class Arms {
	LinearOpMode callingOpMode;
	Hardware hardware;

	public Arms(LinearOpMode opMode, Hardware hw_map) {
		callingOpMode = opMode;
		hardware = hw_map;
	}

	public void update_up_down(Double servo_power) {

		servo_power = Double.max(-1.0, servo_power);
		servo_power = Double.min(1.0, servo_power);

		hardware.armHeightServo.setPower(servo_power);
	}

	public void update_open_closed(Double left_servo_power, Double right_servo_power) {

		left_servo_power = Double.max(-1.0, left_servo_power);
		left_servo_power = Double.min(1.0, left_servo_power);

		right_servo_power = Double.max(-1.0, right_servo_power);
		right_servo_power = Double.min(1.0, right_servo_power);

		hardware.armOpenClosedServo.setPower(left_servo_power);
		hardware.armOpenClosedServo2.setPower(right_servo_power);
	}

	public void update_open_closed(Double servo_power) {

		servo_power = Double.max(-1.0, servo_power);
		servo_power = Double.min(1.0, servo_power);

		hardware.armOpenClosedServo.setPower(servo_power);
		hardware.armOpenClosedServo2.setPower(servo_power);
	}
}
