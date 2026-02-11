package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class StubbedFlywheel extends SubsystemBase {
	private AngularVelocity speed;

	public StubbedFlywheel() {}

	public void startFlywheel(AngularVelocity speed) {
		this.speed = speed;
	}

	public void stopFlywheel() {
		System.out.println("Stopped flywheel");
	}

	public boolean isAtCruiseVelocity() {
		return true;
	}
}
