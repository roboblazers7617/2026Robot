// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

/** Add your docs here. */
public class Hopper extends SubsystemBase {
	private NetworkTable Hopper;
	private NetworkTable Uptake;
	private DoubleEntry HopperSpeedEntry;
	private DoubleEntry UptakeSpeedEntry;
	private DoublePublisher HopperSpeedDisplayPublisher;
	private DoublePublisher UptakeSpeedDisplayPublisher;

	private String file = "output.chrp";

	Orchestra music = new Orchestra();
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
		music.addInstrument(bigSpinny);
		music.addInstrument(littleSpinny);
		var status3 = music.loadMusic(file);
		if (!status3.isOK()) {
			// log error
			System.out.println("AAAAAAAAAAH");
		} else {
			System.out.println("yippppeeeee");
		}
	}

	public void startHopperMotor(double Speed) {
		bigSpinny.set(Speed);
	}

	public void stopHopperMotor() {
		System.out.println("stop hopper");
		bigSpinny.stopMotor();
	}

	public void startUptakeMotor(double Speed) {
		System.out.println("start uptake");
		littleSpinny.set(Speed);
	}

	public void stopUptakeMotor() {
		System.out.println("stop uptake");
		littleSpinny.stopMotor();
	}

	public void stopAll() {
		bigSpinny.stopMotor();
		littleSpinny.stopMotor();
	}

	public void musicTime() {
		music.play();
		System.out.println("working?");
	}

	public void puaseMusicTime() {
		music.pause();
		System.out.println("puased");
	}

	public void stopMusicTime() {
		music.pause();
		System.out.println("stop");
	}

	public Command musicTimeCommand() {
		return runOnce(() -> musicTime());
	}

	public Command musicTimePuaseCommand() {
		return runOnce(() -> puaseMusicTime());
	}

	public Command musicTimeStopCommand() {
		return runOnce(() -> stopMusicTime());
	}

	public Command startHopperMotorCommand(Supplier<Double> Speed) {
		return runOnce(() -> startHopperMotor(Speed.get()));
	}

	public Command stopHopperMotorCommand() {
		return runOnce(() -> stopHopperMotor());
	}

	public Command startUptakeMotorCommand(Supplier<Double> Speed) {
		return runOnce(() -> startUptakeMotor(Speed.get()));
	}

	public Command stopUptakeMotorCommand() {
		return runOnce(() -> stopUptakeMotor());
	}

	public Command stopBothCommand() {
		return runOnce(() -> stopAll());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		HopperSpeedDisplayPublisher.set(HopperSpeedEntry.getAsDouble());
		UptakeSpeedDisplayPublisher.set(UptakeSpeedEntry.getAsDouble());
	}

	private void setupNetworkTables(NetworkTable table) {
		// Create a link to the subtable for the shooter data
		Hopper = table;
		Uptake = table;

		// Display the current speed
		HopperSpeedDisplayPublisher = table.getDoubleTopic("Hopper Speed").publish();
		UptakeSpeedDisplayPublisher = table.getDoubleTopic("Uptake Speed").publish();

		// Create an entry that allows you to vary the speed of the shooter
		HopperSpeedEntry = Hopper.getDoubleTopic(("Hopper Speed")).getEntry(10);
		UptakeSpeedEntry = Uptake.getDoubleTopic(("Uptake Speed")).getEntry(10);
		HopperSpeedEntry.set(20);
		UptakeSpeedEntry.set(20);

		// Put commands on the Dashboard to allow testing different shooter speeds
		// This uses the value from the slider on the Dashboard
		SmartDashboard.putData("Stop Hopper", stopHopperMotorCommand());
		SmartDashboard.putData("Stop Uptake", stopUptakeMotorCommand());
		SmartDashboard.putData("Stop Both", stopBothCommand());
		SmartDashboard.putData("Start Hopper", startHopperMotorCommand(() -> HopperSpeedEntry.getAsDouble()));
		SmartDashboard.putData("Start Uptake", startUptakeMotorCommand(() -> UptakeSpeedEntry.getAsDouble()));
		SmartDashboard.putData("start", musicTimeCommand());
		SmartDashboard.putData("puase", musicTimePuaseCommand());
		SmartDashboard.putData("Stop", musicTimeStopCommand());
	}
}
