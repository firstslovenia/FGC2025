package org.firstinspires.ftc.teamcode.opmodes.hardwaretesting;

public enum HardwareComponent {
	MotorLeftForward,
	MotorRightForward,
	MotorFrontSideways,
	MotorBackSideways,
	MotorLifterLeft,
	MotorLifterRight,
	ServoUpDownLeft,
	ServoUpDownRight,
	ArmsUpDown,
	ArmsOpenClosed,
	Lifter;

	private static final HardwareComponent[] values = values();

	public HardwareComponent next() {
		return values[(this.ordinal() + 1) % values.length];
	}

	public HardwareComponent previous() {
		return values[Math.floorMod(this.ordinal() - 1, values.length)];
	}
}
