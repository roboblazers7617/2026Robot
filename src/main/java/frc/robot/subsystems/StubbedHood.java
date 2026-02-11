package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class StubbedHood extends SubsystemBase {
	private Angle angle;

	public StubbedHood() {}

	public void moveToPosition(Angle angle) {
		this.angle = angle;
	}

	public boolean isAtPosition() {
		return true;
	}
}
