package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StubbedIntakeShoulder extends SubsystemBase {
	public StubbedIntakeShoulder() {}

	public void raiseShoulder() {
		System.out.println("Shoulder raised");
	}

	public void lowerShoulder() {
		System.out.println("Shoulder lowered");
	}

	public boolean isAtTarget() {
		return true;
	}
}
