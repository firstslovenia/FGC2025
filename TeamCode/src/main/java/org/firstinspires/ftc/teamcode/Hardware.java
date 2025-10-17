package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

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
	public static DcMotor lifterMotorLeft = null;
	public static DcMotor lifterMotorRight = null;

	// Arms lower / higher servo
	public static CRServo armHeightServoLeft = null;
	public static CRServo armHeightServoRight = null;

	// Arms open / closed
	public static Servo armOpenClosedServoLeft = null;
	public static Servo armOpenClosedServoRight = null;

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

		lifterMotorLeft = callingOpMode.hardwareMap.get(DcMotor.class, "lifterLeft");
		lifterMotorRight = callingOpMode.hardwareMap.get(DcMotor.class, "lifterRight");

		lifterMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

		armHeightServoLeft = callingOpMode.hardwareMap.get(CRServo.class, "armHeightServoLeft");
		armHeightServoRight = callingOpMode.hardwareMap.get(CRServo.class, "armHeightServoRight");

		armHeightServoRight.setDirection(DcMotorSimple.Direction.REVERSE);

		armOpenClosedServoLeft = callingOpMode.hardwareMap.get(Servo.class, "armOpenClosedServoLeft");
		armOpenClosedServoRight = callingOpMode.hardwareMap.get(Servo.class, "armOpenClosedServoRight");

		armOpenClosedServoLeft.setDirection(Servo.Direction.REVERSE);

		imu = callingOpMode.hardwareMap.get(IMU.class, "imu");
	}
}
