// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

	MotionMagicVelocityVoltage velocity = new MotionMagicVelocityVoltage(0);

	private AngularVelocity setpoint = RadiansPerSecond.zero();

	public HopperUptake() {
		bigSpinny = new TalonFX(HopperConstants.BIG_SPINNY_CAN_ID);
		littleSpinny = new TalonFX(HopperConstants.LITTLE_SPINNY_CAN_ID);
		// The configuration for uptake
		TalonFXConfiguration uptakeConfig = new TalonFXConfiguration();
		// // Neutral mode
		uptakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		// // Current limits
		uptakeConfig.CurrentLimits.StatorCurrentLimit = HopperConstants.UPTAKE_STATOR_CURRENT_LIMIT;
		uptakeConfig.CurrentLimits.SupplyCurrentLimit = HopperConstants.UPTAKE_SUPPLY_CURRENT_LIMIT;
		uptakeConfig.CurrentLimits.SupplyCurrentLowerLimit = HopperConstants.UPTAKE_LOWER_CURRENT_LIMIT;
		uptakeConfig.CurrentLimits.StatorCurrentLimitEnable = HopperConstants.UPTAKE_ENABLE_STATOR_LIMIT;
		uptakeConfig.CurrentLimits.SupplyCurrentLimitEnable = HopperConstants.UPTAKE_ENABLE_SUPPLY_LIMIT;
		// // Velocity voltage motion magic
		uptakeConfig.MotionMagic.MotionMagicAcceleration = HopperConstants.UPTAKE_ACCELERATION;
		uptakeConfig.MotionMagic.MotionMagicCruiseVelocity = HopperConstants.UPTAKE_MAXIMUM_VELOCITY;
		// // Velocity voltage motion magic
		uptakeConfig.MotorOutput.Inverted = HopperConstants.UPTAKE_IS_INVERTED;
		// Uptake PID
		uptakeConfig.Slot0.kP = HopperConstants.UPTAKE_KP;
		uptakeConfig.Slot0.kI = HopperConstants.UPTAKE_KI;
		uptakeConfig.Slot0.kD = HopperConstants.UPTAKE_KD;
		uptakeConfig.Slot0.kV = HopperConstants.UPTAKE_KV;
		uptakeConfig.Slot0.kS = HopperConstants.UPTAKE_KS;
		// The configuration for hopper
		TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
		// // Neutral mode
		hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		// // Current limits
		hopperConfig.CurrentLimits.StatorCurrentLimit = HopperConstants.HOPPER_STATOR_CURRENT_LIMIT;
		hopperConfig.CurrentLimits.SupplyCurrentLimit = HopperConstants.HOPPER_SUPPLY_CURRENT_LIMIT;
		hopperConfig.CurrentLimits.SupplyCurrentLowerLimit = HopperConstants.HOPPER_LOWER_CURRENT_LIMIT;
		hopperConfig.CurrentLimits.StatorCurrentLimitEnable = HopperConstants.HOPPER_ENABLE_STATOR_LIMIT;
		hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = HopperConstants.HOPPER_ENABLE_SUPPLY_LIMIT;
		// // Velocity voltage motion magic
		hopperConfig.MotionMagic.MotionMagicAcceleration = HopperConstants.HOPPER_ACCELERATION;
		hopperConfig.MotionMagic.MotionMagicCruiseVelocity = HopperConstants.HOPPER_MAXIMUM_VELOCITY;
		// // Inverted Value
		hopperConfig.MotorOutput.Inverted = HopperConstants.HOPPER_IS_INVERTED;
		// Hopper PID
		hopperConfig.Slot0.kP = HopperConstants.HOPPER_KP;
		hopperConfig.Slot0.kI = HopperConstants.HOPPER_KI;
		hopperConfig.Slot0.kD = HopperConstants.HOPPER_KD;
		hopperConfig.Slot0.kV = HopperConstants.HOPPER_KV;
		hopperConfig.Slot0.kS = HopperConstants.HOPPER_KS;
		// Try to apply config multiple time. Break after successfully applying
		for (int i = 0; i < 2; ++i) {
			var status = bigSpinny.getConfigurator().apply(uptakeConfig);
			if (status.isOK()) {
				break;
			}
		}
		for (int i = 0; i < 2; ++i) {
			var status = littleSpinny.getConfigurator().apply(hopperConfig);
			if (status.isOK()) {
				break;
			}
		}
	}

	public boolean isUptakeAtTarget() {
		// System.out.println("Uptake target " + bigSpinny.getVelocity().getValueAsDouble() + " " + setpoint.in(RotationsPerSecond) + " " + HopperConstants.TOLERANCE.in(RotationsPerSecond));
		return MathUtil.isNear(setpoint.in(RotationsPerSecond), bigSpinny.getVelocity().getValueAsDouble(), HopperConstants.TOLERANCE.in(RotationsPerSecond));
	}

	private void startHopperMotorRPM(AngularVelocity RPS) {
		littleSpinny.setControl(velocity.withVelocity(RPS));
	}

	private void startUptakeMotorRPM(AngularVelocity RPS) {
		bigSpinny.setControl(velocity.withVelocity(RPS));
		setpoint = RPS;
	}

	public void startHopperForward() {
		startHopperMotorRPM(HopperConstants.FORWARD_HOPPER_RPS);
	}

	public void startUptakeForward() {
		startUptakeMotorRPM(HopperConstants.FORWARD_UPTAKE_RPS);
	}

	public void startHopperUnjam() {
		startHopperMotorRPM(HopperConstants.BACKWARD_HOPPER_RPS);
	}

	public void startUptakeUnjam() {
		startUptakeMotorRPM(HopperConstants.BACKWARD_UPTAKE_RPS);
	}

	public void stopHopper() {
		littleSpinny.stopMotor();
	}

	// TODO: I'm wondering if we need separate stopUptake and stopHopper functions as we would always
	// need to stop both. Right? At least we can't stop uptake if hopper is not also stopped.
	public void stopUptake() {
		bigSpinny.stopMotor();
		setpoint = RadiansPerSecond.zero();
	}

	public void startAllMotors() {
		startHopperMotorRPM(HopperConstants.FORWARD_HOPPER_RPS);
		startUptakeMotorRPM(HopperConstants.FORWARD_UPTAKE_RPS);
	}

	public void startMotorsUnjam() {
		stopHopper();
		startUptakeMotorRPM(HopperConstants.BACKWARD_UPTAKE_RPS);
	}

	public void stopBothMotors() {
		littleSpinny.stopMotor();
		bigSpinny.stopMotor();
	}

	// COMMANDS
	public Command startUptakeForwarCommand() {
		return runOnce(() -> startUptakeForward());
	}

	public Command startBothCommand() {
		return startUptakeForwarCommand().andThen(Commands.waitUntil(() -> isUptakeAtTarget()).finallyDo(this::startHopperForward));
	}

	public Command startUnJamCommand() {
		return runOnce(() -> startMotorsUnjam());
	}

	public Command stopBothMotorsCommand() {
		return runOnce(() -> stopBothMotors());
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("at target", isUptakeAtTarget());
	}
}
