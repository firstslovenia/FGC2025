package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

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

	// Motors which turns our rope lifter
	public static DcMotor lifterMotor1 = null;
	public static DcMotor lifterMotor2 = null;

	public static IMU imu = null;

	public Hardware (LinearOpMode opmode) {
		callingOpMode = opmode;
	}

	public void init()    {
		frontSidewaysMotor = callingOpMode.hardwareMap.get(DcMotor.class, "frontSideways");
		rightForwardMotor = callingOpMode.hardwareMap.get(DcMotor.class, "rightForward");
		leftForwardMotor = callingOpMode.hardwareMap.get(DcMotor.class, "leftForward");
		backSidewaysMotor = callingOpMode.hardwareMap.get(DcMotor.class, "backSideways");

		frontSidewaysMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		rightForwardMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		leftForwardMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		backSidewaysMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		// Make the positive direction on motor (+x, +y) --> up and to the right, like a cartesian coordinate system
		// (This is assuming the motors spin clockwise and their directions are flipped once by gears)
		frontSidewaysMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		leftForwardMotor.setDirection(DcMotorSimple.Direction.REVERSE);

		lifterMotor1 = callingOpMode.hardwareMap.get(DcMotor.class, "lifter1");
		lifterMotor2 = callingOpMode.hardwareMap.get(DcMotor.class, "lifter2");

		lifterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

		// Lets not fall off the rope when we aren't actively climbing higher
		lifterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		lifterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		imu = callingOpMode.hardwareMap.get(IMU.class, "imu");
	}
}
