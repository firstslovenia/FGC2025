package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Vector2D;

/// Helps test if your odometry calibration is correct
///
/// Should complete 10 (n) full turns and end on the same heading
@TeleOp(name = "CalibrationValidator")
public class CalibrationValidator extends LinearOpMode {

	Hardware hardware;
	Drivetrain drivetrain;
	Odometry odometry;

	/// Number of turns to do
	int n_turns = 10;

	/// Number of turns done
	int n_turns_done = 0;

	/// The odometry heading offset from the last loop, to check if we've completed a full circle
	double last_odometry_heading = 0.0;

	@Override
	public void runOpMode() {

		hardware = new Hardware(this);
		hardware.init();

		odometry = new Odometry(this, hardware);

		drivetrain = new Drivetrain(this, hardware);
		drivetrain.fieldCentricRotation = false;
		drivetrain.rotationMultiplier = 0.5;

		waitForStart();

		while (opModeIsActive()) {
			telemetry.addLine("-- Calibration Validator --");
			telemetry.addData("turns done", n_turns_done);
			telemetry.addData("turns to do", n_turns);
			telemetry.addData("heading (degs)", Math.toDegrees(odometry.heading));

			// Spin counterclockwise
			if (n_turns_done < n_turns) {
				drivetrain.update(new Vector2D(0.0, 0.0), new Vector2D(1.0, 0.0));
			} else {
				drivetrain.update(new Vector2D(), new Vector2D());
			}

			odometry.update();
			telemetry.update();

			// If we had a sudden jump, we probably jumped from (lots of rotation) to (no rotation), meaning we completed a turn
			if (Math.abs(odometry.heading - last_odometry_heading) > Math.PI) {
				n_turns_done += 1;
			}

			last_odometry_heading = odometry.heading;
		}
	}
}
