package frc.robot.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.Timer;

@CustomLoggerFor(Timer.class)
public class TimerLogger extends ClassSpecificLogger<Timer> {
	public TimerLogger() {
		super(Timer.class);
	}

	@Override
	protected void update(EpilogueBackend backend, Timer timer) {
		backend.log("Running", timer.isRunning());
		backend.log("Time", timer.get());
	}
}
