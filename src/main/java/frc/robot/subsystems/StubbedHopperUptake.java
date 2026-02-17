package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class StubbedHopperUptake extends SubsystemBase {
	private final Timer timer = new Timer();
	private final DIOSim uptakeBeamBreakSim;

	public StubbedHopperUptake(DIOSim uptakeBeamBreakSim) {
		this.uptakeBeamBreakSim = uptakeBeamBreakSim;
	}

	@Override
	public void simulationPeriodic() {
		uptakeBeamBreakSim.setValue(isHopperEmpty());
	}

	public void startHopperForward() {
		System.out.println("Started hopper");
		timer.restart();
	}

	public void startHopperUnjam() {
		System.out.println("Started hopper unjamming");
	}

	public void stopHopper() {
		System.out.println("Stopped hopper");
		timer.stop();
	}

	public void startUptakeForward() {
		System.out.println("Started uptake");
	}

	public void startUptakeUnjam() {
		System.out.println("Started uptake unjamming");
	}

	public void stopUptake() {
		System.out.println("Stopped uptake");
	}

	public boolean isHopperEmpty() {
		return timer.hasElapsed(3);
	}
}
