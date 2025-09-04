package org.firstinspires.ftc.teamcode;

/// Defines possible driving states of the robot
public enum State {
	/// We are just driving around, no usage of lifter
	Drive,
	/// Both driving and lifter
	CatchRope,
	/// Only lifter
	Climb,
}
