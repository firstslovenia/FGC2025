package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/// Our lifter logic.
///
/// A motor turns some gears which in-turn slowly rotate a thingy which catches and spools the rope.
public class Lifter {
	LinearOpMode callingOpMode;
	Hardware hardware;

	public Lifter(LinearOpMode opMode, Hardware hw_map) {
		callingOpMode = opMode;
		hardware = hw_map;
	}

	/// Called to enable the motors, setup what we need to be able to use the lifter
	public void initialize() {
		hardware.lifterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		hardware.lifterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	/// Called to undo initialize
	public void uninitialize() {
		hardware.lifterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		hardware.lifterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
	}

	public void update(Double climb_up_power) {

		climb_up_power = Double.max(-1.0, climb_up_power);
		climb_up_power = Double.min(1.0, climb_up_power);

		hardware.lifterMotor1.setPower(climb_up_power);
		hardware.lifterMotor2.setPower(climb_up_power);
	}
}
