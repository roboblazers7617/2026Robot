// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.HopperConstants;

/** Add your docs here. */
public class Hopper extends SubsystemBase {
	private NetworkTable Hopper;
	private DoubleEntry shooterSpeedEntry;
	private DoublePublisher HopperSpeedDisplayPublisher;

	// Motors
	private final TalonFX bigSpinny;
	private final TalonFX littleSpinny;

	public Hopper(NetworkTable table) {
		setupNetworkTables(table);
		bigSpinny = new TalonFX(HopperConstants.BIG_SPINNY_CAN_ID);
		littleSpinny = new TalonFX(HopperConstants.LITTLE_SPINNY_CAN_ID);
		TalonFXConfiguration config = new TalonFXConfiguration();
		// Put's the motor in Coast mode to make it easier to move by hand
		config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		config.Slot0.kS = HopperConstants.KS; // Static gain
		config.Slot0.kV = HopperConstants.KV; // Velocity gain
		config.Slot0.kP = HopperConstants.KP; // Proportional gain
		config.MotionMagic.MotionMagicCruiseVelocity = HopperConstants.CRUISE_VELOCITY; // Max velocity
		config.MotionMagic.MotionMagicAcceleration = HopperConstants.ACCELERATION; // Max acceleration allowed
		// Try to apply config multiple time. Break after successfully applying
		for (int i = 0; i < 2; ++i) {
			var status = bigSpinny.getConfigurator().apply(config);
			var status2 = littleSpinny.getConfigurator().apply(config);
			if (status.isOK() && status2.isOK())
				break;
		}
	}

	public void startHopperMotor(double Speed) {
		bigSpinny.set(Speed);
	}

	public void stopHopperMotor() {
		bigSpinny.stopMotor();
	}

	public void startUptakeMotor(double Speed) {
		littleSpinny.set(Speed);
	}

	public void stopUptakeMotor() {
		littleSpinny.stopMotor();
	}

	public void stopAll() {
		bigSpinny.stopMotor();
		littleSpinny.stopMotor();
	}

	public Command StartHopperMotorCommand(Supplier<Double> Speed) {
		return runOnce(() -> startHopperMotor(Speed.get()));
	}

	public Command StopHopperMotorCommand() {
		return runOnce(() -> stopHopperMotor());
	}

	public Command StartUptakeMotorCommand(Supplier<Double> Speed) {
		return runOnce(() -> startUptakeMotor(Speed.get()));
	}

	public Command StopUptakeMotorCommand() {
		return runOnce(() -> stopUptakeMotor());
	}

	public Command StopBothCommand() {
		return runOnce(() -> stopAll());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		HopperSpeedDisplayPublisher.set(shooterSpeedEntry.getAsDouble());
	}

	private void setupNetworkTables(NetworkTable table) {
		// Create a link to the subtable for the shooter data
		Hopper = table;

		// Display the current speed
		HopperSpeedDisplayPublisher = table.getDoubleTopic("speedDisplay").publish();

		// Create an entry that allows you to vary the speed of the shooter
		shooterSpeedEntry = Hopper.getDoubleTopic(("shooterSpeed")).getEntry(10);
		shooterSpeedEntry.set(20);

		// Put commands on the Dashboard to allow testing different shooter speeds
		// This uses the value from the slider on the Dashboard
		SmartDashboard.putData("Stop Hopper", StopHopperMotorCommand());
		SmartDashboard.putData(" Stop Uptake", StopUptakeMotorCommand());
		SmartDashboard.putData("Stop Both", StopBothCommand());
		SmartDashboard.putData("Start Hopper", StartHopperMotorCommand(() -> shooterSpeedEntry.getAsDouble()));
		SmartDashboard.putData("Start Uptake", StartUptakeMotorCommand(() -> shooterSpeedEntry.getAsDouble()));
	}
}
