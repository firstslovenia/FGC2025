package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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

	// Arms lower / higher servo
	public static CRServo armHeightServo = null;
	public static CRServo armHeightServo2 = null;

	// Arms open / closed
	public static CRServo armOpenClosedServo = null;
	public static CRServo armOpenClosedServo2 = null;

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

		lifterMotor1 = callingOpMode.hardwareMap.get(DcMotor.class, "lifterLeft");
		lifterMotor2 = callingOpMode.hardwareMap.get(DcMotor.class, "lifterRight");

		lifterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

		armHeightServo = callingOpMode.hardwareMap.get(CRServo.class, "armHeightServoLeft");
		armHeightServo2 = callingOpMode.hardwareMap.get(CRServo.class, "armHeightServoRight");

		armHeightServo2.setDirection(DcMotorSimple.Direction.REVERSE);

		armOpenClosedServo = callingOpMode.hardwareMap.get(CRServo.class, "armOpenClosedServoLeft");
		armOpenClosedServo2 = callingOpMode.hardwareMap.get(CRServo.class, "armOpenClosedServoRight");

		armOpenClosedServo2.setDirection(DcMotorSimple.Direction.REVERSE);

		imu = callingOpMode.hardwareMap.get(IMU.class, "imu");
	}
}
