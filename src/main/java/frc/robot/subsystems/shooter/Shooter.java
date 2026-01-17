// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
	private final TalonFX motor;
	// Velocity output control for the flywheel
	private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);

	/** Creates a new Shooter. */
	public Shooter() {
		motor = new TalonFX(ShooterConstants.CAN_ID);

		TalonFXConfiguration config = new TalonFXConfiguration();
		// Put's the motor in Coast mode to make it easier to move by hand
		config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		// Configure the motor to make sure positive voltage is counter clockwise
		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		config.Slot0.kS = ShooterConstants.KS; // Static gain
		config.Slot0.kV = ShooterConstants.KV; // Velocity gain
		config.Slot0.kP = ShooterConstants.KP; // Proportional gain
		config.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.CRUISE_VELOCITY; // Max velocity
		config.MotionMagic.MotionMagicAcceleration = ShooterConstants.ACCELERATION; // Max acceleration allowed
		// Try to apply config multiple time. Break after successfully applying
		for (int i = 0; i < 2; ++i) {
			var status = motor.getConfigurator().apply(config);
			if (status.isOK())
				break;
		}
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	private void startShooter(double speed) {
		motor.setControl(velocityOut.withVelocity(speed));
	}

	private void stopShooter() {
		motor.stopMotor();
	}

	public Command StartShooterCommand(Supplier<Double> speed) {
		return runOnce(() -> startShooter(speed.get()));
	}

	public Command StopShooterCommand() {
		return runOnce(() -> stopShooter());
	}
}
