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
	//  ║  front     front ⌄ ║
	//  ║                    ║
	//  ║ ⌃ left       right ║
	//  ║ | back       back  ║
	//  ║ ⌄             <--> ║
	//  ╚════════════════════╝

	public static DcMotor leftFrontMotor = null;
	public static DcMotor rightFrontMotor = null;
	public static DcMotor leftBackMotor = null;
	public static DcMotor rightBackMotor= null;

	public Hardware (LinearOpMode opmode) {
		callingOpMode = opmode;
	}

	public void init()    {
		leftFrontMotor  = callingOpMode.hardwareMap.get(DcMotor.class, "leftFront");
		rightFrontMotor  = callingOpMode.hardwareMap.get(DcMotor.class, "rightFront");
		leftBackMotor  = callingOpMode.hardwareMap.get(DcMotor.class, "leftBack");
		rightBackMotor  = callingOpMode.hardwareMap.get(DcMotor.class, "rightBack");

		// Make the positive direction on motor (+x, +y) --> up and to the right, like a cartesian coordinate system
		// (This is assuming the motors spin clockwise and their directions are flipped once by gears)
		leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
	}
}
