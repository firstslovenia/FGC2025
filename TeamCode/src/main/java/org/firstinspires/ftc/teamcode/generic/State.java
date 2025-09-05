package org.firstinspires.ftc.teamcode.generic;

import org.firstinspires.ftc.teamcode.opmodes.main.StateEnum;

public interface State {

	/// Called when this state become the active one
	void on_switch_from(StateEnum old_state);

	/// Called when this state is no longer the active one
	void on_switch_to(StateEnum new_state);

	/// Called every loop this state is active
	void loop();
}
