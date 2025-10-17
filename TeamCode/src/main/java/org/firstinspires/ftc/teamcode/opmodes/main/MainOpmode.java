package org.firstinspires.ftc.teamcode.opmodes.main;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Arms;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Lifter;
import org.firstinspires.ftc.teamcode.generic.State;

import java.util.HashMap;

@TeleOp(name = "Main")
public class MainOpmode extends LinearOpMode {

	Hardware hardware;
	Drivetrain drivetrain;
	Lifter lifter;
	Arms arms;

	HashMap<StateEnum, State> states = new HashMap<>();
	StateEnum current_state_key = StateEnum.Drive;

	// Keeps track of when the next and previous state buttons were pressed, for debouncing
	boolean next_state_button_last = false;

	long next_state_button_pressed_time = 0;
	boolean previous_state_button_last = false;
	long previous_state_button_pressed_time = 0;

	/// The minimum interval of pressed that we consider not being a bounce
	static long button_bounce_ms = 200;

	/// Changes the current state to the one provided, returning it
	State switch_state_to(State current_state_object, StateEnum new_state_key) {
		current_state_object.on_switch_to(new_state_key);

		StateEnum old_state_key = current_state_key;
		current_state_key = new_state_key;

		State next_state = states.get(current_state_key);
		assert next_state != null;

		next_state.on_switch_from(old_state_key);

		return next_state;
	}

	@Override
	public void runOpMode() {

		waitForStart();

		hardware = new Hardware(this);
		hardware.init();

		drivetrain = new Drivetrain(this, hardware);
		drivetrain.fieldCentricTranslation = true;
		drivetrain.fieldCentricRotation = true;
		drivetrain.resetStartingDirection();

		lifter = new Lifter(this, hardware);

		arms = new Arms(this, hardware);

		// Create states
		states.put(StateEnum.Drive, new DriveState(this, hardware, drivetrain, arms));
		states.put(StateEnum.CatchRope, new CatchRopeState(this, hardware, drivetrain, arms, lifter));
		states.put(StateEnum.Climb, new ClimbState(this, hardware, arms, lifter));

		while (opModeIsActive()) {

			State current_state_class = states.get(current_state_key);
			assert current_state_class != null;

			long now = System.currentTimeMillis();
			long since_next_state_button_pressed_ms = now - next_state_button_pressed_time;
			long since_previous_state_button_pressed_ms = now - previous_state_button_pressed_time;

			if (gamepad1.y && !next_state_button_last && since_next_state_button_pressed_ms > button_bounce_ms) {
				next_state_button_pressed_time = now;

				switch (current_state_key) {
					case Drive:
						current_state_class = switch_state_to(current_state_class, StateEnum.CatchRope);
						break;

					case CatchRope:
						current_state_class = switch_state_to(current_state_class, StateEnum.Climb);
						break;

					default:
						break;
				}
			}

			if (gamepad1.x && !previous_state_button_last && since_previous_state_button_pressed_ms > button_bounce_ms) {
				previous_state_button_pressed_time = now;

				switch (current_state_key) {
					case Climb:
						current_state_class = switch_state_to(current_state_class, StateEnum.CatchRope);
						break;

					case CatchRope:
						current_state_class = switch_state_to(current_state_class, StateEnum.Drive);
						break;

					default:
						break;
				}
			}

			next_state_button_last = gamepad1.y;
			previous_state_button_last = gamepad1.x;

			telemetry.addData("state", current_state_key.toString());

			current_state_class.loop();

			telemetry.update();
		}
	}
}
