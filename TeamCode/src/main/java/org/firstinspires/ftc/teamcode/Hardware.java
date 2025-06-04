package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Hardware {

	private LinearOpMode callingOpMode = null;

	//  Motor configuration:
	//  ╔═════════════════════╗
	//  ║ <-->              ⌃ ║
	//  ║ front       right | ║
	//  ║ sideways  forward ⌄ ║
	//  ║                     ║
	//  ║ ⌃ left         back ║
	//  ║ | forward  sideways ║
	//  ║ ⌄              <--> ║
	//  ╚═════════════════════╝

	public static DcMotor frontSidewaysMotor = null;
	public static DcMotor rightForwardMotor = null;
	public static DcMotor leftForwardMotor = null;
	public static DcMotor backSidewaysMotor = null;

	public Hardware (LinearOpMode opmode) {
		callingOpMode = opmode;
	}

	public void init()    {
		frontSidewaysMotor = callingOpMode.hardwareMap.get(DcMotor.class, "frontSideways");
		rightForwardMotor = callingOpMode.hardwareMap.get(DcMotor.class, "rightForward");
		leftForwardMotor = callingOpMode.hardwareMap.get(DcMotor.class, "leftForward");
		backSidewaysMotor = callingOpMode.hardwareMap.get(DcMotor.class, "backSideways");

		// Make the positive direction on motor (+x, +y) --> up and to the right, like a cartesian coordinate system
		// (This is assuming the motors spin clockwise and their directions are flipped once by gears)
		frontSidewaysMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		leftForwardMotor.setDirection(DcMotorSimple.Direction.REVERSE);
	}
}
