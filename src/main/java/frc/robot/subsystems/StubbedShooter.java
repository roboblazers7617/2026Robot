package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class StubbedShooter extends SubsystemBase {
	private AngularVelocity speed;

	public StubbedShooter() {}

	public void startShooter(AngularVelocity speed, boolean motionMagic) {
		this.speed = speed;
	}

	public boolean isAtTarget() {
		return true;
	}
}
