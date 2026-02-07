// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.sim.MovingAverageFilterSim;
import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

@Logged
public class Shooter extends SubsystemBase {
	private final TalonFX leaderMotor;
	private final TalonFX followermotor;
	private final TalonFX hoodMotor;
	// Velocity output control for the flywheel
	private final MotionMagicVelocityVoltage velocityMotionMagic = new MotionMagicVelocityVoltage(0);
	private final VelocityVoltage velocityOut = new VelocityVoltage(0);
	private DoubleEntry shooterSpeedEntry;
	private NetworkTable shooterTable;

	private final DutyCycleEncoder hoodEncoder;

	/** Creates a new Shooter. */
	public Shooter(NetworkTable table) {
		setupNetworkTable(table);

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
		shooterConfig.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.CRUISE_VELOCITY; // Max velocity
		shooterConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.ACCELERATION; // Max acceleration allowed

		// Try to apply config multiple time. Break after successfully applying
		for (int i = 0; i < 2; ++i) {
			var status = leaderMotor.getConfigurator().apply(shooterConfig);
			if (status.isOK())
				break;
		}

		private final MotionMagicVoltage positionOut = new MotionMagicVoltage(0);

		hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_CAN_ID);

		TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

		hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		// Configure the motor to make sure positive voltage is counter clockwise
		hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		hoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Use cosine gravity compensation
		hoodConfig.Slot0.kG = 0.0; // Gravity gain
		hoodConfig.Slot0.kS = 0.0; // Static gain
		hoodConfig.Slot0.kP = 0.0; // Proportional gain
		hoodConfig.Slot0.kD = 0.0; // Derivative gain
		hoodConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

		// Try to apply config multiple time. Break after successfully applying
		for (int i = 0; i < 2; ++i) {
			var status = hoodMotor.getConfigurator().apply(hoodConfig);
			if (status.isOK())
				break;
		}

		hoodEncoder = new DutyCycleEncoder(0);
		hoodEncoder.setDutyCycleRange(0.003884, 0.99806);
	}

	private void setPosition(Angle position, boolean motionMagic) {
		if (motionMagic) {
			hoodMotor.setControl(velocityMotionMagic.withPosition(position));
		} else {
			hoodMotor.setControl(velocityMotionMagic.withPosition(position));
		}
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void startFlywheel(double speed, boolean motionMagic) {
		if (motionMagic) {
			leaderMotor.setControl(velocityMotionMagic.withVelocity(speed));
		} else {
			leaderMotor.setControl(velocityOut.withVelocity(speed));
		}
		System.out.println("Speed is" + speed);
	}

	public void stopFlywheel() {
		leaderMotor.stopMotor();
	}

	public Command startFlywheelCommand(Supplier<Double> speed) {
		return runOnce(() -> startFlywheel(speed.get(), true));
	}

	public Command stopFlywheelCommand() {
		return runOnce(() -> stopFlywheel());
	}

	private void setupNetworkTable(NetworkTable table) {
		shooterTable = table;
		shooterSpeedEntry = shooterTable.getDoubleTopic(("shooterVelocity")).getEntry(10);
		shooterSpeedEntry.set(10);
	}

	public Command startHoodCommand(Supplier<Angle> angle) {
		return runOnce(() -> {});
	}

	public Command stopHoodCommand(Supplier<Angle> angle) {
		return runOnce(() -> {});
	}

	public void setPosition(double position) {
		// Apply the position output to the leader motor
		hoodMotor.setControl(positionOut.withPosition(position));
	}
}