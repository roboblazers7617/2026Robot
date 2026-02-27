package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;

public class DrivetrainControls {

	/* Setting up bindings for necessary control of the swerve drive platform */
	public final SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
			.withDeadband(DrivetrainConstants.MAX_SPEED * 0.1)
			.withHeadingPID(6, 0, 0.1)
			.withRotationalDeadband(DrivetrainConstants.MaxAngularRate * 0.1) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	public final SwerveRequest.FieldCentric spin = new SwerveRequest.FieldCentric()
			.withRotationalDeadband(DrivetrainConstants.MaxAngularRate * 0.1) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // se open-loop control for drive motors

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

	public Command defualtSwerveCommand() {
		return drivetrain.applyRequest(() -> {
			if (Math.abs(driverController.getRightY()) >= 0.5 || Math.abs(driverController.getRightX()) >= 0.5) {
				drivetrainControls.drive.withTargetDirection(new Rotation2d(-driverController.getRightY(), -driverController.getRightX()));
			}

			return drivetrainControls.drive.withVelocityX(-driverController.getLeftY() * DrivetrainConstants.MAX_SPEED * drivetrainControls.speedMultiplier) // Drive forward with negative Y (forward)
					.withVelocityY(-driverController.getLeftX() * DrivetrainConstants.MAX_SPEED * drivetrainControls.speedMultiplier);// Drive left with negative X (left)
		});
	}
}
