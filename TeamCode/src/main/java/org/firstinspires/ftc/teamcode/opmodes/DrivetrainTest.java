package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name = "DrivetrainTest")
public class DrivetrainTest extends LinearOpMode {

	Hardware hardware;
	Drivetrain drivetrain;

	@Override
	public void runOpMode() {

		hardware = new Hardware(this);
		hardware.init();

		drivetrain = new Drivetrain(hardware);

		waitForStart();

		while (opModeIsActive()) {
			Pair<Double, Double> magnitude_and_direction = Drivetrain.getMagnitudeAndPhiFor(gamepad1.left_stick_x, gamepad1.left_stick_y);

			// Gamepads like to give us both x and y between 0 and 1, meaning the length can be between 0 and sqrt(2)
			double power = magnitude_and_direction.first / Math.sqrt(2);
			double direction = magnitude_and_direction.second;

			drivetrain.update(power, direction, gamepad1.right_stick_x);
		}
	}
}
