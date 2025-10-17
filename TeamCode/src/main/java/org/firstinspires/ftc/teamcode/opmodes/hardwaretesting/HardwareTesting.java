package org.firstinspires.ftc.teamcode.opmodes.hardwaretesting;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name = "Hardware Testing")
public class HardwareTesting extends LinearOpMode {

	Hardware hardware;

	HardwareComponent selected_component = HardwareComponent.values()[0];

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

		while (opModeIsActive()) {

				float power = gamepad1.right_stick_y;

				switch(selected_component) {
					case MotorLeftForward:
						hardware.leftForwardMotor.setPower(power);
						break;
					case MotorRightForward:
						hardware.rightForwardMotor.setPower(power);
						break;
					case MotorFrontSideways:
						hardware.frontSidewaysMotor.setPower(power);
						break;
					case MotorBackSideways:
						hardware.backSidewaysMotor.setPower(power);
						break;
					case MotorLifterLeft:
						hardware.lifterMotorLeft.setPower(power);
						break;
					case MotorLifterRight:
						hardware.lifterMotorRight.setPower(power);
						break;
					case ServoUpDownLeft:
						hardware.armHeightServoLeft.setPower(power);
						break;
					case ServoUpDownRight:
						hardware.armHeightServoRight.setPower(power);
						break;
					case ServoOpenClosedLeft:
						hardware.armOpenClosedServoLeft.setPower(power);
						break;
					case ServoOpenClosedRight:
						hardware.armOpenClosedServoRight.setPower(power);
						break;
				}

				long now = System.currentTimeMillis();
				long since_next_button_pressed_ms = now - next_button_pressed_time;
				long since_previous_button_pressed_ms = now - previous_button_pressed_time;

				if (gamepad1.a && !next_button_last && since_next_button_pressed_ms > button_bounce_ms) {
					selected_component = selected_component.next();
					next_button_pressed_time = now;
				}

				if (gamepad1.b && !previous_button_last && since_previous_button_pressed_ms > button_bounce_ms) {
					selected_component = selected_component.previous();
					previous_button_pressed_time = now;
				}

				next_button_last = gamepad1.a;
				previous_button_last = gamepad1.b;

				telemetry.addData("selected component", selected_component);
				telemetry.addData("power", power);
				telemetry.update();
		}
	}
}
