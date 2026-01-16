// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

/** Add your docs here. */
public class DrivetrainControls {
	/**
	 * The Drivetrain to control.
	 */
	private final Drivetrain drivetrain;
	/**
	 * Speed multiplier used for scaling controller inputs (0, 1].
	 */
	private double speedMultiplier;

	/**
	 * Creates a new DrivetrainControls.
	 *
	 * @param drivetrain
	 *            The Drivetrain to control.
	 */
	public DrivetrainControls(Drivetrain drivetrain) {
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
	}

	/**
	 * Sets the controller speed multiplier back to {@link GeneratedConstants#TRANSLATION_SCALE_NORMAL}.
	 */
	public void resetSpeedMultiplier() {
		// setSpeedMultiplier();
	}
}
