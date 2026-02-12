package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Radians;

@Logged
public class StubbedHood extends SubsystemBase {
	private Angle angle = Radians.zero();

	public StubbedHood() {}

	public void moveToPosition(Angle angle) {
		this.angle = angle;
	}

	public boolean isAtPosition() {
		return true;
	}
}
