// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;

/** Add your docs here. */
public class DrivetrainControls {
	// /**
	// * The Drivetrain to control.
	// */
	// private final Drivetrain drivetrain;
	// /**
	// * Speed multiplier used for scaling controller inputs (0, 1].
	// */
	// private double speedMultiplier;

	// private double MaxSpeed = 1.0 * GeneratedConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
	// private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

	// private final SwerveRequest.FieldCentric driveAngular = new SwerveRequest.FieldCentric()
	// .withDeadband(OperatorConstants.DEADBAND)
	// .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
	// .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	// private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	// private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
	// private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
	// .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
	// private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle();

	// /**
	// * Creates a new DrivetrainControls.
	// *
	// * @param drivetrain
	// * The Drivetrain to control.
	// */
	// public DrivetrainControls(Drivetrain drivetrain) {
	// this.drivetrain = drivetrain;

	// // Set things to their default states
	// resetSpeedMultiplier();
	// }

	// /**
	// * Sets the controller speed multiplier.
	// *
	// * @param speedMultiplier
	// * Multiplier to set (0, 1].
	// */
	// public void setSpeedMultiplier(double speedMultiplier) {
	// this.speedMultiplier = speedMultiplier;
	// }

	// /**
	// * Sets the controller speed multiplier back to {@link DrivetrainConstants#TRANSLATION_SCALE_NORMAL}.
	// */
	// public void resetSpeedMultiplier() {
	// setSpeedMultiplier(DrivetrainConstants.TRANSLATION_SCALE_NORMAL);
	// }

	// /**
	// * Sets the controller speed multiplier. Resets the multiplier when canceled.
	// *
	// * @param speedMultiplier
	// * Multiplier to set (0, 1].
	// * @return
	// * Command to run.
	// */
	// public Command setSpeedMultiplierCommand(Supplier<Double> speedMultiplier) {
	// return Commands.run(() -> setSpeedMultiplier(speedMultiplier.get()))
	// .finallyDo(this::resetSpeedMultiplier);
	// }

	// /**
	// * Drive the robot given a SwerveInputStream, scaled with the {@link #speedMultiplier}.
	// *
	// * @param inputStream
	// * The {@link SwerveInputStream} to read from.
	// * @param orientation
	// * The translation's field of reference.
	// * @return
	// * Command to run.
	// * @see Drivetrain#drive(ChassisSpeeds, TranslationOrientation)
	// */
	// private Command driveInputStreamScaledCommand(SwerveInputStream inputStream, TranslationOrientation orientation) {
	// return drivetrain.run(() -> {
	// inputStream.allianceRelativeControl(orientation == TranslationOrientation.FdIELD_RELATIVE);
	// inputStream.scaleTranslation(speedMultiplier);
	// drivetrain.drive(inputStream.get(), orientation);
	// });
	// }

	// /**
	// * Converts driver input into a SwerveInputStream with default settings.
	// *
	// * @param controller
	// * The controller to read from.
	// * @return
	// * SwerveInputStream with data from the controller.
	// */
	// // private SwerveRequest driveGeneric(CommandXboxController controller) {
	// // return SwerveInputStream.of(drivetrain.getSwerveDrive(), () -> (-1 * controller.getLeftY()), () -> (-1 * controller.getLeftX()))
	// // .deadband(OperatorConstants.DEADBAND);
	// // }

	// /**
	// * A copy of {@link #driveGeneric(CommandXboxController)} that uses angular velocity control for turning.
	// *
	// * @param controller
	// * The controller to read from.
	// * @return
	// * SwerveInputStream with data from the controller.
	// */
	// public SwerveRequest driveAngularVelocity(CommandXboxController controller) {
	// // return driveGeneric(controller)
	// // .withControllerRotationAxis(() -> (-1 * controller.getRightX()));
	// return driveAngular.withVelocityX(-controller.getLeftY() * speedMultiplier)
	// .withVelocityY(-controller.getLeftX() * speedMultiplier)
	// .withRotationalRate(-controller.getRightX());
	// }

	// /**
	// * A copy of {@link #driveGeneric(CommandXboxController)} that uses heading control for turning.
	// *
	// * @param controller
	// * The controller to read from.
	// * @return
	// * SwerveInputStream with data from the controller.
	// */
	// public SwerveRequest driveDirectAngle(CommandXboxController controller) {
	// return
	// driveFacingAngle.withControllerHeadingAxis(() -> (-1 * controller.getRightX()), () -> (-1 * controller.getRightY()))
	// .headingWhile(true);
	// }

	// /**
	// * A copy of {@link #driveGeneric(CommandXboxController)} that pulls rotation from controller axis 2 for use in simulation.
	// *
	// * @param controller
	// * The controller to read from.
	// * @return
	// * SwerveInputStream with data from the controller.
	// */
	// public SwerveInputStream driveDirectAngleSim(CommandXboxController controller) {
	// return driveGeneric(controller)
	// .withControllerRotationAxis(() -> controller.getRawAxis(2))
	// .withControllerHeadingAxis(() -> Math.sin(controller.getRawAxis(2) * Math.PI) * (Math.PI * 2), () -> Math.cos(controller.getRawAxis(2) * Math.PI) * (Math.PI * 2))
	// .headingWhile(true);
	// }

	// /**
	// * A copy of {@link #driveGeneric(CommandXboxController)} that uses a preset heading.
	// *
	// * @param controller
	// * The controller to read from.
	// * @param heading
	// * The rotation to point the heading to.
	// * @return
	// * SwerveInputStream with data from the controller.
	// */
	// public SwerveInputStream driveStaticHeading(CommandXboxController controller, Supplier<Rotation2d> heading) {
	// return driveGeneric(controller)
	// .withControllerHeadingAxis(() -> heading.get().getSin() * (Math.PI * 2), () -> heading.get().getCos() * (Math.PI * 2))
	// .headingWhile(true);
	// }

	// /**
	// * {@link #driveInputStreamScaledCommand(SwerveInputStream, TranslationOrientation)} that uses {@link #driveAngularVelocity(CommandXboxController)}. Calls {@link Drivetrain#resetLastAngleScalar()} on end to prevent snapback.
	// *
	// * @param controller
	// * Controller to use.
	// * @param orientation
	// * The translation's field of reference.
	// * @return
	// * Command to run.
	// */
	// public Command driveAngularVelocityCommand(CommandXboxController controller, TranslationOrientation orientation) {
	// return driveInputStreamScaledCommand(driveAngularVelocity(controller), orientation)
	// .finallyDo(drivetrain::resetLastAngleScalar);
	// }

	// /**
	// * {@link #driveInputStreamScaledCommand(SwerveInputStream, TranslationOrientation)} that uses {@link DrivetrainControls#driveDirectAngle(CommandXboxController)}.
	// *
	// * @param controller
	// * Controller to use.
	// * @param orientation
	// * The translation's field of reference.
	// * @return
	// * Command to run.
	// */
	// public Command driveDirectAngleCommand(CommandXboxController controller, TranslationOrientation orientation) {
	// return driveInputStreamScaledCommand(driveDirectAngle(controller), orientation);
	// }

	// /**
	// * {@link #driveInputStreamScaledCommand(SwerveInputStream, TranslationOrientation)} that uses {@link DrivetrainControls#driveDirectAngleSim(CommandXboxController)}.
	// *
	// * @param controller
	// * Controller to use.
	// * @param orientation
	// * The translation's field of reference.
	// * @return
	// * Command to run.
	// */
	// public Command driveDirectAngleSimCommand(CommandXboxController controller, TranslationOrientation orientation) {
	// return driveInputStreamScaledCommand(driveDirectAngleSim(controller), orientation);
	// }

	// /**
	// * {@link #driveInputStreamScaledCommand(SwerveInputStream, TranslationOrientation)} that uses {@link DrivetrainControls#driveStaticHeading(CommandXboxController, Supplier)}.
	// *
	// * @param controller
	// * Controller to use.
	// * @param orientation
	// * The translation's field of reference.
	// * @param heading
	// * The rotation to point the heading to.
	// * @return
	// * Command to run.
	// */
	// public Command driveStaticHeadingCommand(CommandXboxController controller, TranslationOrientation orientation, Supplier<Rotation2d> heading) {
	// return driveInputStreamScaledCommand(driveStaticHeading(controller, heading), orientation)
	// .finallyDo(() -> drivetrain.setLastAngleScalar(heading));
	// }

}