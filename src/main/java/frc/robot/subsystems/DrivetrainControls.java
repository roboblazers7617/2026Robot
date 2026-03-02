package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainControls {
	public final SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
			.withDeadband(DrivetrainConstants.MAX_SPEED_DEADBAND * DrivetrainConstants.DRIVE_DEADBAND)
			.withHeadingPID(DrivetrainConstants.HEADING_kP, DrivetrainConstants.HEADING_ki, DrivetrainConstants.HEADING_kd)
			.withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE_DEADBAND * DrivetrainConstants.ROTATIONAL_DEADBAND)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	// swerve request for regular spinny (the defualt this year)
	public final SwerveRequest.FieldCentric spin = new SwerveRequest.FieldCentric()
			.withDeadband(DrivetrainConstants.MAX_SPEED_DEADBAND * DrivetrainConstants.DRIVE_DEADBAND)
			.withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE_DEADBAND * DrivetrainConstants.ROTATIONAL_DEADBAND)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	public final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

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
	private void setSpeedMultiplier(double speedMultiplier) {
		this.speedMultiplier = speedMultiplier;
	}

	/**
	 * Sets the controller speed multiplier back to normal
	 */
	private void resetSpeedMultiplier() {
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
