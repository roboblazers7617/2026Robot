package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainControls {
	public final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	public final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	/**
	 * The Drivetrain to control.
	 */
	private final CommandSwerveDrivetrain drivetrain;
	/**
	 * Speed multiplier used for scaling controller inputs (0, 1].
	 */
	public double speedMultiplier = DrivetrainConstants.NORMAL_SPEED_MULTIPLIER;

	/**
	 * Creates a new DrivetrainControls.
	 *
	 * @param drivetrain
	 *            The Drivetrain to control.
	 */
	public DrivetrainControls(CommandSwerveDrivetrain drivetrain) {
		this.drivetrain = drivetrain;

		// Set things to their default states
		resetSpeedMultiplier();
	}

	/**
	 * Sets the controller speed multiplier.
	 *
	 * @param speedMultiplier
	 *            Multiplier to set (0, 1].
	 */
	public void setSpeedMultiplier(double speedMultiplier) {
		this.speedMultiplier = speedMultiplier;
		System.out.println(speedMultiplier);
	}

	/**
	 * Sets the controller speed multiplier back to {@link DrivetrainConstants#TRANSLATION_SCALE_NORMAL}.
	 */
	public void resetSpeedMultiplier() {
		setSpeedMultiplier(DrivetrainConstants.NORMAL_SPEED_MULTIPLIER);
	}

	/**
	 * Sets the controller speed multiplier. Resets the multiplier when canceled.
	 *
	 * @param speedMultiplier
	 *            Multiplier to set (0, 1].
	 * @return
	 *         Command to run.
	 */
	public Command setSpeedMultiplierCommand(Supplier<Double> speedMultiplier) {
		return Commands.run(() -> setSpeedMultiplier(speedMultiplier.get()))
				.finallyDo(this::resetSpeedMultiplier);
	}
}
