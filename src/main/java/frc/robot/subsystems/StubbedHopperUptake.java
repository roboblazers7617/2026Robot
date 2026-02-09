package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class StubbedHopperUptake extends SubsystemBase {
	public StubbedHopperUptake() {}

	public void start() {
		System.out.println("Started hopper/uptake");
	}

	public void stop() {
		System.out.println("Stopped hopper/uptake");
	}
}
