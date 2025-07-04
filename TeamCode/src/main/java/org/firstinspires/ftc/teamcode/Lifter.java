package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

	public void update(Double climb_up_power) {

		climb_up_power = Double.max(-1.0, climb_up_power);
		climb_up_power = Double.min(1.0, climb_up_power);

		hardware.lifterMotor1.setPower(climb_up_power);
	}
}
