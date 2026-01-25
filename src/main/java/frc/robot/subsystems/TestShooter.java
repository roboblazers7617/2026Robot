// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.HopperConstants;

public class TestShooter extends SubsystemBase {
	private NetworkTable shooterTable;
	// private final DoubleTopic shooterSpeedTopic;
	private DoubleEntry shooterSpeedEntry;
	private DoublePublisher speedDisplayPublisher;

	private final TalonFX leaderMotor;
	private final TalonFX followermotor;

	// Velocity output control for the flywheel
	private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);

	public TestShooter(NetworkTable table) {
		setupNetworkTables(table);

		leaderMotor = new TalonFX(HopperConstants.LEADER_CAN_ID);
		followermotor = new TalonFX(HopperConstants.FOLLOWER_CAN_ID);
		followermotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
		TalonFXConfiguration config = new TalonFXConfiguration();
		// Put's the motor in Coast mode to make it easier to move by hand
		config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		// Configure the motor to make sure positive voltage is counter clockwise
		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		config.Slot0.kS = HopperConstants.KS; // Static gain
		config.Slot0.kV = HopperConstants.KV; // Velocity gain
		config.Slot0.kP = HopperConstants.KP; // Proportional gain
		config.MotionMagic.MotionMagicCruiseVelocity = HopperConstants.CRUISE_VELOCITY; // Max velocity
		config.MotionMagic.MotionMagicAcceleration = HopperConstants.ACCELERATION; // Max acceleration allowed
		// Try to apply config multiple time. Break after successfully applying
		for (int i = 0; i < 2; ++i) {
			var status = leaderMotor.getConfigurator().apply(config);
			if (status.isOK())
				break;
		}
	}

	private void startShooter(double speed) {
		leaderMotor.setControl(velocityOut.withVelocity(speed));
		System.out.println("Speed is" + speed);
	}

	private void stopShooter() {
		leaderMotor.stopMotor();
	}

	public Command StartShooterCommand(Supplier<Double> speed) {
		return runOnce(() -> startShooter(speed.get()));
	}

	public Command StopShooterCommand() {
		return runOnce(() -> stopShooter());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		speedDisplayPublisher.set(shooterSpeedEntry.getAsDouble());
	}

	private void setupNetworkTables(NetworkTable table) {
		// Create a link to the subtable for the shooter data
		shooterTable = table;

		// Display the current speed
		speedDisplayPublisher = table.getDoubleTopic("speedDisplay").publish();

		// Create an entry that allows you to vary the speed of the shooter
		shooterSpeedEntry = shooterTable.getDoubleTopic(("shooterSpeed")).getEntry(10);
		shooterSpeedEntry.set(20);

		// Put commands on the Dashboard to allow testing different shooter speeds
		// This uses the value from the slider on the Dashboard
		SmartDashboard.putData("Stop Shooter", StopShooterCommand());
		SmartDashboard.putData("Start Shooter", StartShooterCommand(() -> shooterSpeedEntry.getAsDouble()));
	}
}
