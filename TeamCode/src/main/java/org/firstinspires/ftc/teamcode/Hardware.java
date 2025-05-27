package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Hardware {

	private LinearOpMode callingOpMode = null;

	//  Motor configuration:
	//  ╔════════════════════╗
	//  ║ <-->             ⌃ ║
	//  ║  left      right | ║
	//  ║  Front     Front ⌄ ║
	//  ║                    ║
	//  ║ ⌃ left       right ║
	//  ║ | Back       back  ║
	//  ║ ⌄             <--> ║
	//  ╚════════════════════╝

	static DcMotor leftFront = null;
	static DcMotor rightFront = null;
	static DcMotor leftBack = null;
	static DcMotor rightBack = null;

	public Hardware (LinearOpMode opmode) {
		callingOpMode = opmode;
	}

	public void init()    {
		leftFront  = callingOpMode.hardwareMap.get(DcMotor.class, "leftFront");
		rightFront  = callingOpMode.hardwareMap.get(DcMotor.class, "rightFront");
		leftBack  = callingOpMode.hardwareMap.get(DcMotor.class, "leftBack");
		rightBack  = callingOpMode.hardwareMap.get(DcMotor.class, "rightBack");

		// Make the positive direction on motor (+x, +y) --> up and to the right, like a cartesian coordinate system
		// (This is assuming the motors spin clockwise and their directions are flipped once by gears)
		leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
		rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
	}
}
