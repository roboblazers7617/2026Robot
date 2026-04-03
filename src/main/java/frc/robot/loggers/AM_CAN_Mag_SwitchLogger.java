package frc.robot.loggers;

import com.andymark.jni.AM_CAN_Mag_Switch;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(AM_CAN_Mag_Switch.class)
public class AM_CAN_Mag_SwitchLogger extends ClassSpecificLogger<AM_CAN_Mag_Switch> {
	public AM_CAN_Mag_SwitchLogger() {
		super(AM_CAN_Mag_Switch.class);
	}

	@Override
	protected void update(EpilogueBackend backend, AM_CAN_Mag_Switch magSwitch) {
		AM_CAN_Mag_Switch.AM_MagSwitchData data = magSwitch.getData();

		backend.log("Magnet Detected", data.magnetDetected);
	}
}
