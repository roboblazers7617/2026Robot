// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Shooter extends SubsystemBase {
	private final TalonFX leaderMotor;
	private final TalonFX followermotor;
	// Velocity output control for the flywheel
	private final VelocityVoltage velocityOut = new VelocityVoltage(0);
	private NetworkTable shooterTable;
	private DoubleEntry shooterSpeedEntry;
	private BooleanEntry isShooterSpeedAtTargetEntry;
	private AngularVelocity requestedSpeed;

	/** Creates a new Shooter. */
	public Shooter(NetworkTable table) {
		setupNetworkTable(table);
		requestedSpeed = RadiansPerSecond.zero();

		leaderMotor = new TalonFX(ShooterConstants.LEADER_CAN_ID);
		followermotor = new TalonFX(ShooterConstants.FOLLOWER_CAN_ID);
		followermotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));

		TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
		// Put's the motor in Coast mode to make it easier to move by hand
		shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		// Configure the motor to make sure positive voltage is counter clockwise
		shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		shooterConfig.Slot0.kS = ShooterConstants.KS; // Static gain
		shooterConfig.Slot0.kV = ShooterConstants.KV; // Velocity gain
		shooterConfig.Slot0.kP = ShooterConstants.KP; // Proportional gain
		shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = ShooterConstants.SUPPLY_CURRENT_LOWER_LIMIT; // Limit to 40A
		shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT;
		shooterConfig.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants.SUPPLY_CURRENT_LOWER_TIME; // For longer than 0.1 seconds
		shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.SUPPLY_CURRENT_LIMIT_ENABLE; // Enable the limit
		// Try to apply config multiple time. Break after successfully applying
		for (int i = 0; i < 2; ++i) {
			var status = leaderMotor.getConfigurator().apply(shooterConfig);
			if (status.isOK())
				break;
		}
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		isShooterSpeedAtTargetEntry.set(isAtTarget());
	}

	public boolean isAtTarget() {
		return leaderMotor.getVelocity()
				.getValue()
				.isNear(requestedSpeed, ShooterConstants.TOLERANCE);
	}

	public void startFlywheel(AngularVelocity speed) {
		leaderMotor.setControl(velocityOut.withVelocity(speed));

		System.out.println("Speed is" + speed);
	}

	public void stopFlywheel() {
		leaderMotor.stopMotor();
	}

	public Command startFlywheelCommand(Supplier<AngularVelocity> speed) {
		return runOnce(() -> startFlywheel(speed.get()));
	}

	public Command stopFlywheelCommand() {
		return runOnce(() -> stopFlywheel());
	}

	private void setupNetworkTable(NetworkTable table) {
		shooterTable = table;
		shooterSpeedEntry = shooterTable.getDoubleTopic(("shooterVelocity")).getEntry(10);
		shooterSpeedEntry.set(10);
		isShooterSpeedAtTargetEntry = shooterTable.getBooleanTopic(("shooter is at target speed")).getEntry(false);
		isShooterSpeedAtTargetEntry.set(false);
	}
}
