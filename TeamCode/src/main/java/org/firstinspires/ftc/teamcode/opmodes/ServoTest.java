package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name = "ServoCancer")
public class ServoTest extends LinearOpMode {

	Hardware hardware;

	boolean next_button_last = false;
	long next_button_pressed_time = 0;
	boolean previous_button_last = false;
	long previous_button_pressed_time = 0;

	/// The minimum interval of pressed that we consider not being a bounce
	static long button_bounce_ms = 200;

	@Override
	public void runOpMode() {

		hardware = new Hardware(this);
		hardware.init();

		waitForStart();

		hardware.armOpenClosedServoLeft.setPosition(0.0);

			while (opModeIsActive()) {

				long now = System.currentTimeMillis();
				long since_next_button_pressed_ms = now - next_button_pressed_time;
				long since_previous_button_pressed_ms = now - previous_button_pressed_time;

				if (gamepad1.a && !next_button_last && since_next_button_pressed_ms > button_bounce_ms) {
					hardware.armOpenClosedServoRight.setPosition(hardware.armOpenClosedServoRight.getPosition() + 0.1);
					next_button_pressed_time = now;
				}

				if (gamepad1.b && !previous_button_last && since_previous_button_pressed_ms > button_bounce_ms) {
					hardware.armOpenClosedServoRight.setPosition(hardware.armOpenClosedServoRight.getPosition() - 0.1);
					previous_button_pressed_time = now;
				}

				next_button_last = gamepad1.a;
				previous_button_last = gamepad1.b;

				telemetry.addData("o/c  left pos", hardware.armOpenClosedServoLeft.getPosition());
				telemetry.addData("o/c right pos", hardware.armOpenClosedServoRight.getPosition());
				telemetry.update();
			}
	}
}
