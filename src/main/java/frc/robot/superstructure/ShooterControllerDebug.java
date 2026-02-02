package frc.robot.superstructure;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Degrees;

/**
 * A debug controller for the shooter controller. Allows for manually setting shooter speed, turret angle, and hood angle.
 */
public class ShooterControllerDebug {
	private final ShooterController shooterController;

	private final DoubleEntry shooterSpeedEntry;
	private final DoubleEntry turretAngleEntry;
	private final DoubleEntry hoodAngleEntry;

	/**
	 * Creates a new ShooterControllerDebug.
	 *
	 * @param shooterController
	 *            The ShooterController to control.
	 */
	public ShooterControllerDebug(ShooterController shooterController) {
		this.shooterController = shooterController;

		final NetworkTable table = NetworkTableInstance.getDefault()
				.getTable("Debug/Shooter Controller");

		shooterSpeedEntry = table.getDoubleTopic("Shooter Speed")
				.getEntry(0.0);
		shooterSpeedEntry.set(0.0);

		turretAngleEntry = table.getDoubleTopic("Turret Angle")
				.getEntry(0.0);
		turretAngleEntry.set(0.0);

		hoodAngleEntry = table.getDoubleTopic("Hood Angle")
				.getEntry(0.0);
		hoodAngleEntry.set(0.0);

		// Set up dashboard
		SmartDashboard.putData("Shooter Controller/Debug/Commit Shooter Values", commitShooterValuesCommand());
	}

	/**
	 * Command to set the shooter values to the values specified on NetworkTables.
	 *
	 * @return
	 *         Command to run.
	 */
	private Command commitShooterValuesCommand() {
		return shooterController.setStateCommand(() -> new ShooterValues()
				.setShooterSpeed(RPM.of(shooterSpeedEntry.get()))
				.setTurretAngle(Degrees.of(turretAngleEntry.get()))
				.setHoodAngle(Degrees.of(hoodAngleEntry.get())))
				.ignoringDisable(true);
	}
}
