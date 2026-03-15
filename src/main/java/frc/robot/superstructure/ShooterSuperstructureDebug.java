package frc.robot.superstructure;

import java.util.Optional;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.superstructure.sources.ShootingSource;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Degrees;

/**
 * A debug controller for the shooter superstructure. Allows for manually setting shooter speed, hood angle, and turret angle.
 */
public class ShooterSuperstructureDebug {
	private final ShooterSuperstructure shooterSuperstructure;

	private final DoubleEntry flywheelSpeedEntry;
	private final DoubleEntry hoodAngleEntry;
	private final DoubleEntry turretAngleEntry;

	/**
	 * Creates a new ShooterSuperstructureDebug.
	 *
	 * @param shooterSuperstructure
	 *            The ShooterSuperstructure to control.
	 */
	public ShooterSuperstructureDebug(ShooterSuperstructure shooterSuperstructure) {
		this.shooterSuperstructure = shooterSuperstructure;

		final NetworkTable table = NetworkTableInstance.getDefault()
				.getTable(SuperstructureConstants.SHOOTER_SUPERSTRUCTURE_TABLE_NAME)
				.getSubTable("Debug");

		flywheelSpeedEntry = table.getDoubleTopic("Flywheel Speed")
				.getEntry(0.0);
		flywheelSpeedEntry.set(0.0);

		hoodAngleEntry = table.getDoubleTopic("Hood Angle")
				.getEntry(0.0);
		hoodAngleEntry.set(0.0);

		turretAngleEntry = table.getDoubleTopic("Turret Angle")
				.getEntry(0.0);
		turretAngleEntry.set(0.0);

		// Set up dashboard
		SmartDashboard.putData(SuperstructureConstants.SHOOTER_SUPERSTRUCTURE_TABLE_NAME + "/Debug/Start Debug Control", startDebugControlCommand());
	}

	/**
	 * Command to start debug control. This sets the shooting value source to an internal debug value source.
	 *
	 * @return
	 *         Command to run.
	 */
	private Command startDebugControlCommand() {
		return shooterSuperstructure.setSourceCommand(new ShootingSourceDebug());
	}

	/**
	 * A shooting source that just provides the values from the Debug class.
	 */
	private class ShootingSourceDebug extends ShootingSource {
		public ShootingSourceDebug() {
			super("Debug");
		}

		public Optional<ShooterValues> get() {
			ShooterValues values = new ShooterValues()
					.setFlywheelSpeed(RPM.of(flywheelSpeedEntry.get()))
					.setTurretAngle(Degrees.of(turretAngleEntry.get()))
					.setHoodAngle(Degrees.of(hoodAngleEntry.get()));

			return Optional.of(values);
		}
	}
}
