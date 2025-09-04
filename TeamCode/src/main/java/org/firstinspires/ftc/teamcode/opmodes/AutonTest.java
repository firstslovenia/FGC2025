package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.generic.Vector2D;

@Autonomous(name = "AutonTest")
public class AutonTest extends LinearOpMode {

	Hardware hardware;
	Drivetrain drivetrain;

	@Override
	public void runOpMode() {
		hardware = new Hardware(this);
		hardware.init();

		drivetrain = new Drivetrain(this, hardware);
		drivetrain.fieldCentricRotation = false;
		drivetrain.fieldCentricTranslation = true;

		waitForStart();

		while (opModeIsActive()) {
			drivetrain.update(new Vector2D(0.0, 0.0), new Vector2D(1.0, 0.0));
			telemetry.update();
		}
	}
}
