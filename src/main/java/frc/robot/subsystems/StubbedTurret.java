package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Radians;

@Logged
public class StubbedTurret extends SubsystemBase {
	private Angle angle = Radians.zero();

	public StubbedTurret() {}

	public void setPosition(Angle angle, boolean motionMagic) {
		this.angle = angle;
	}

	public boolean isAtTarget() {
		return true;
	}

	public void unspool() {
		System.out.println("Unspooled turret");
	}
}
