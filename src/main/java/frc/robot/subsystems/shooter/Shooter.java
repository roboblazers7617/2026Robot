// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SignalConstants;

@Logged
public class Shooter extends SubsystemBase {
	private final TalonFX leaderMotor;
	private final TalonFX followermotor;
	// Velocity output control for the flywheel
	private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
	private final MotionMagicVelocityVoltage mmvelocityVoltage = new MotionMagicVelocityVoltage(0).withSlot(1);
	private NetworkTable shooterTable;
	private DoubleEntry shooterSpeedEntry;
	private AngularVelocity requestedSpeed;

	/** Creates a new Shooter. */
	public Shooter(NetworkTable table) {
		setupNetworkTable(table);
		requestedSpeed = RadiansPerSecond.zero();

		leaderMotor = new TalonFX(ShooterConstants.LEADER_CAN_ID);
		followermotor = new TalonFX(ShooterConstants.FOLLOWER_CAN_ID);
		followermotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));

		TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
		// Put's the motor in Coast mode to make it easier to move by hand
		shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		// Configure the motor to make sure positive voltage is counter clockwise
		shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		shooterConfig.Slot0.kS = ShooterConstants.KS_0; // Static gain
		shooterConfig.Slot0.kV = ShooterConstants.KV_0; // Velocity gain
		shooterConfig.Slot0.kP = ShooterConstants.KP_0; // Proportional gain
		shooterConfig.Slot0.kD = ShooterConstants.KD_0;
		shooterConfig.Slot1.kS = ShooterConstants.KS_1; // Static gain
		shooterConfig.Slot1.kV = ShooterConstants.KV_1; // Velocity gain
		shooterConfig.Slot1.kP = ShooterConstants.KP_1; // Proportional gain
		shooterConfig.Slot1.kD = ShooterConstants.KD_1;
		shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = ShooterConstants.SUPPLY_CURRENT_LOWER_LIMIT; // Limit to 40A
		shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT;
		shooterConfig.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants.SUPPLY_CURRENT_LOWER_TIME; // For longer than 0.1 seconds
		shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.SUPPLY_CURRENT_LIMIT_ENABLE; // Enable the limit
		shooterConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.ACCELERATION;
		shooterConfig.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.MAXIMUM_VELOCITY;
		// Try to apply config multiple time. Break after successfully applying
		for (int i = 0; i < 2; ++i) {
			var status = leaderMotor.getConfigurator().apply(shooterConfig);
			if (status.isOK())
				break;
		}

		// Set up the status signal update frequencies
		leaderMotor.getVelocity().setUpdateFrequency(SignalConstants.POLLED_POSITION_UPDATE_FREQUENCY);
		leaderMotor.getTorqueCurrent().setUpdateFrequency(SignalConstants.OUTPUT_UPDATE_FREQUENCY);
		leaderMotor.getStatorCurrent().setUpdateFrequency(SignalConstants.OUTPUT_UPDATE_FREQUENCY);
		leaderMotor.getMotorVoltage().setUpdateFrequency(SignalConstants.OUTPUT_UPDATE_FREQUENCY);
		leaderMotor.optimizeBusUtilization(SignalConstants.OPTIMIZED_UPDATE_FREQUENCY);

		followermotor.getVelocity().setUpdateFrequency(SignalConstants.POLLED_POSITION_UPDATE_FREQUENCY);
		followermotor.getTorqueCurrent().setUpdateFrequency(SignalConstants.OUTPUT_UPDATE_FREQUENCY);
		followermotor.getStatorCurrent().setUpdateFrequency(SignalConstants.OUTPUT_UPDATE_FREQUENCY);
		followermotor.getMotorVoltage().setUpdateFrequency(SignalConstants.OUTPUT_UPDATE_FREQUENCY);
		followermotor.optimizeBusUtilization(SignalConstants.OPTIMIZED_UPDATE_FREQUENCY);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public boolean isAtTarget() {
		return MathUtil.isNear(requestedSpeed.in(RotationsPerSecond), leaderMotor.getVelocity().getValueAsDouble(), ShooterConstants.TOLERANCE.in(RotationsPerSecond));
	}

	public void startFlywheel(AngularVelocity speed) {
		leaderMotor.setControl(mmvelocityVoltage.withVelocity(speed));
		requestedSpeed = speed;
		// System.out.println("Speed is" + speed);
	}

	public void stopFlywheel() {
		leaderMotor.stopMotor();
		requestedSpeed = RotationsPerSecond.of(0);
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
	}
}
