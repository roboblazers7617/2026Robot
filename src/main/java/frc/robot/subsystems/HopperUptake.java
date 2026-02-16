// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

/** Add your docs here. */
public class HopperUptake extends SubsystemBase {

	// Motors private final TalonFX bigSpinny;
	/**
	 * The Hopper motor
	 */
	private final TalonFX littleSpinny;
	/**
	 * the Intake motor
	 */
	private final TalonFX bigSpinny;

	VelocityVoltage velocity = new VelocityVoltage(0);

	private final DigitalInput isTrippedBeamBreak;

	public HopperUptake() {
		isTrippedBeamBreak = new DigitalInput(HopperConstants.BEAM_BREAK_DIO_PIN);
		bigSpinny = new TalonFX(HopperConstants.BIG_SPINNY_CAN_ID);
		littleSpinny = new TalonFX(HopperConstants.LITTLE_SPINNY_CAN_ID);
		// The configuration for uptake
		TalonFXConfiguration uptakeConfig = new TalonFXConfiguration();
		uptakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		uptakeConfig.CurrentLimits.StatorCurrentLimit = HopperConstants.UPTAKE_SUPPLY_CURRENT_LIMIT;
		uptakeConfig.CurrentLimits.SupplyCurrentLimit = HopperConstants.UPTAKE_SUPPLY_CURRENT_LIMIT;
		uptakeConfig.CurrentLimits.SupplyCurrentLowerLimit = HopperConstants.UPTAKE_LOWER_CURRENT_LIMIT;
		uptakeConfig.CurrentLimits.StatorCurrentLimitEnable = HopperConstants.UPTAKE_ENABLE_STATOR_LIMIT;
		uptakeConfig.CurrentLimits.SupplyCurrentLimitEnable = HopperConstants.UPTAKE_ENABLE_SUPPLY_LIMIT;
		// Uptake PID
		uptakeConfig.Slot0.kP = HopperConstants.UPTAKE_KP;
		uptakeConfig.Slot0.kI = HopperConstants.UPTAKE_KI;
		uptakeConfig.Slot0.kD = HopperConstants.UPTAKE_KD;
		uptakeConfig.Slot0.kV = HopperConstants.UPTAKE_KV;
		uptakeConfig.Slot0.kS = HopperConstants.UPTAKE_KS;
		// The configuration for hopper
		TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
		hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		hopperConfig.CurrentLimits.StatorCurrentLimit = HopperConstants.HOPPER_SUPPLY_CURRENT_LIMIT;
		hopperConfig.CurrentLimits.SupplyCurrentLimit = HopperConstants.HOPPER_SUPPLY_CURRENT_LIMIT;
		hopperConfig.CurrentLimits.SupplyCurrentLowerLimit = HopperConstants.HOPPER_LOWER_CURRENT_LIMIT;
		hopperConfig.CurrentLimits.StatorCurrentLimitEnable = HopperConstants.HOPPER_ENABLE_STATOR_LIMIT;
		hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = HopperConstants.HOPPER_ENABLE_SUPPLY_LIMIT;
		// Hopper PID
		hopperConfig.Slot0.kP = HopperConstants.HOPPER_KP;
		hopperConfig.Slot0.kI = HopperConstants.HOPPER_KI;
		hopperConfig.Slot0.kD = HopperConstants.HOPPER_KD;
		hopperConfig.Slot0.kV = HopperConstants.HOPPER_KV;
		hopperConfig.Slot0.kS = HopperConstants.HOPPER_KS;
		// Try to apply config multiple time. Break after successfully applying
		for (int i = 0; i < 2; ++i) {
			var status = bigSpinny.getConfigurator().apply(uptakeConfig);
			var status2 = littleSpinny.getConfigurator().apply(hopperConfig);
			if (status.isOK() && status2.isOK())
				break;
		}
	}

	public boolean isTripped() {
		return !isTrippedBeamBreak.get();
	}

	public void startHopperMotor(double RPM) {
		littleSpinny.setControl(velocity.withVelocity(RPM)); // 80 rotations/sec
	}

	public void startUptakeMotor(double RPM) {
		bigSpinny.setControl(velocity.withVelocity(RPM));
	}

	public void stopAllMotors() {
		bigSpinny.stopMotor();
		littleSpinny.stopMotor();
	}

	public void startAllMotors() {
		startHopperMotor(HopperConstants.FORWARD_HOPPER_RPM);
		startUptakeMotor(HopperConstants.FORWARD_UPTAKE_RPM);
	}

	public void startMotorsUnjam() {
		startHopperMotor(HopperConstants.BACKWARD_HOPPER_RPM);
		startUptakeMotor(HopperConstants.BACKWARD_UPTAKE_RPM);
	}

	public Command startBothCommand() {
		return runOnce(() -> startAllMotors());
	}

	public Command stopBothCommand() {
		return runOnce(() -> stopAllMotors());
	}

	public Command startUnJamCommand() {
		return runOnce(() -> startMotorsUnjam());
	}

	public Command stopBeamBreakCommand() {
		return startBothCommand().andThen(Commands.waitUntil(() -> isTrippedBeamBreak.get()))
				.finallyDo(this::stopAllMotors);
	}

	@Override
	public void periodic() {}
}
