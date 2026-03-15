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

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperUptakeConstants;

/** This class covers the Hopper/Spindexer motor and the Uptake motor */
@Logged
public class HopperUptake extends SubsystemBase {

	// Motors private final TalonFX bigSpinny;
	/**
	 * The Hopper motor tallon x60
	 * Mechanical calls this Spindexer
	 */
	private final TalonFX littleSpinny;
	/**
	 * the Intake motor tallon x60
	 */
	private final TalonFX bigSpinny;

	MotionMagicVelocityVoltage velocity = new MotionMagicVelocityVoltage(0);

	private AngularVelocity setpoint = RadiansPerSecond.zero();

	/**
	 * Keeps track of whether or not the hopper is running forwards (basically are we shooting). Mostly exists just for sim and logging.
	 */
	private boolean isHopperRunningForwards = false;

	public HopperUptake() {
		bigSpinny = new TalonFX(HopperUptakeConstants.BIG_SPINNY_CAN_ID);
		littleSpinny = new TalonFX(HopperUptakeConstants.LITTLE_SPINNY_CAN_ID);
		// The configuration for uptake
		TalonFXConfiguration uptakeConfig = new TalonFXConfiguration();
		// // Neutral mode
		uptakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		// // Current limits
		uptakeConfig.CurrentLimits.StatorCurrentLimit = HopperUptakeConstants.UPTAKE_STATOR_CURRENT_LIMIT;
		uptakeConfig.CurrentLimits.SupplyCurrentLimit = HopperUptakeConstants.UPTAKE_SUPPLY_CURRENT_LIMIT;
		uptakeConfig.CurrentLimits.SupplyCurrentLowerLimit = HopperUptakeConstants.UPTAKE_LOWER_CURRENT_LIMIT;
		uptakeConfig.CurrentLimits.StatorCurrentLimitEnable = HopperUptakeConstants.UPTAKE_ENABLE_STATOR_LIMIT;
		uptakeConfig.CurrentLimits.SupplyCurrentLimitEnable = HopperUptakeConstants.UPTAKE_ENABLE_SUPPLY_LIMIT;
		// // Velocity voltage motion magic
		uptakeConfig.MotionMagic.MotionMagicAcceleration = HopperUptakeConstants.UPTAKE_ACCELERATION;
		uptakeConfig.MotionMagic.MotionMagicCruiseVelocity = HopperUptakeConstants.UPTAKE_MAXIMUM_VELOCITY;
		// // Velocity voltage motion magic
		uptakeConfig.MotorOutput.Inverted = HopperUptakeConstants.UPTAKE_IS_INVERTED;
		// Uptake PID
		uptakeConfig.Slot0.kP = HopperUptakeConstants.UPTAKE_KP;
		uptakeConfig.Slot0.kI = HopperUptakeConstants.UPTAKE_KI;
		uptakeConfig.Slot0.kD = HopperUptakeConstants.UPTAKE_KD;
		uptakeConfig.Slot0.kV = HopperUptakeConstants.UPTAKE_KV;
		uptakeConfig.Slot0.kS = HopperUptakeConstants.UPTAKE_KS;
		// The configuration for hopper
		TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
		// // Neutral mode
		hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		// // Current limits
		hopperConfig.CurrentLimits.StatorCurrentLimit = HopperUptakeConstants.HOPPER_STATOR_CURRENT_LIMIT;
		hopperConfig.CurrentLimits.SupplyCurrentLimit = HopperUptakeConstants.HOPPER_SUPPLY_CURRENT_LIMIT;
		hopperConfig.CurrentLimits.SupplyCurrentLowerLimit = HopperUptakeConstants.HOPPER_LOWER_CURRENT_LIMIT;
		hopperConfig.CurrentLimits.StatorCurrentLimitEnable = HopperUptakeConstants.HOPPER_ENABLE_STATOR_LIMIT;
		hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = HopperUptakeConstants.HOPPER_ENABLE_SUPPLY_LIMIT;
		// // Velocity voltage motion magic
		hopperConfig.MotionMagic.MotionMagicAcceleration = HopperUptakeConstants.HOPPER_ACCELERATION;
		hopperConfig.MotionMagic.MotionMagicCruiseVelocity = HopperUptakeConstants.HOPPER_MAXIMUM_VELOCITY;
		// // Inverted Value
		hopperConfig.MotorOutput.Inverted = HopperUptakeConstants.HOPPER_IS_INVERTED;
		// Hopper PID
		hopperConfig.Slot0.kP = HopperUptakeConstants.HOPPER_KP;
		hopperConfig.Slot0.kI = HopperUptakeConstants.HOPPER_KI;
		hopperConfig.Slot0.kD = HopperUptakeConstants.HOPPER_KD;
		hopperConfig.Slot0.kV = HopperUptakeConstants.HOPPER_KV;
		hopperConfig.Slot0.kS = HopperUptakeConstants.HOPPER_KS;
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
		return MathUtil.isNear(setpoint.in(RotationsPerSecond), bigSpinny.getVelocity().getValueAsDouble(), HopperUptakeConstants.TOLERANCE.in(RotationsPerSecond));
	}

	private void startHopperMotorRPM(AngularVelocity RPS) {
		littleSpinny.setControl(velocity.withVelocity(RPS));
	}

	private void startUptakeMotorRPM(AngularVelocity RPS) {
		bigSpinny.setControl(velocity.withVelocity(RPS));
		setpoint = RPS;
	}

	public void startHopperForward() {
		startHopperMotorRPM(HopperUptakeConstants.FORWARD_HOPPER_RPS);
		isHopperRunningForwards = true;
	}

	public void startUptakeForward() {
		startUptakeMotorRPM(HopperUptakeConstants.FORWARD_UPTAKE_RPS);
	}

	public void startHopperUnjam() {
		startHopperMotorRPM(HopperUptakeConstants.BACKWARD_HOPPER_RPS);
		isHopperRunningForwards = false;
	}

	public void startUptakeUnjam() {
		startUptakeMotorRPM(HopperUptakeConstants.BACKWARD_UPTAKE_RPS);
	}

	public void stopHopper() {
		littleSpinny.stopMotor();
		isHopperRunningForwards = false;
	}

	// TODO: I'm wondering if we need separate stopUptake and stopHopper functions as we would always
	// need to stop both. Right? At least we can't stop uptake if hopper is not also stopped.
	public void stopUptake() {
		bigSpinny.stopMotor();
		setpoint = RadiansPerSecond.zero();
	}

	public void startAllMotors() {
		startHopperMotorRPM(HopperUptakeConstants.FORWARD_HOPPER_RPS);
		startUptakeMotorRPM(HopperUptakeConstants.FORWARD_UPTAKE_RPS);
	}

	public void startMotorsUnjam() {
		stopHopper();
		startUptakeMotorRPM(HopperUptakeConstants.BACKWARD_UPTAKE_RPS);
	}

	public void stopBothMotors() {
		littleSpinny.stopMotor();
		bigSpinny.stopMotor();
	}

	// COMMANDS
	public Command startUptakeForwardCommand() {
		return runOnce(() -> startUptakeForward());
	}

	public Command startBothCommand() {
		return startUptakeForwardCommand().andThen(Commands.waitUntil(() -> isUptakeAtTarget()).finallyDo(this::startHopperForward));
	}

	public Command startUnJamCommand() {
		return runOnce(() -> startMotorsUnjam());
	}

	public Command stopBothMotorsCommand() {
		return runOnce(() -> stopBothMotors());
	}

	/**
	 * Returns true if the hopper is running forwards, false otherwise. This mostly exists for sim.
	 */
	public boolean getIsHopperRunningForwards() {
		return isHopperRunningForwards;
	}

	@Override
	public void periodic() {}
}
