// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

@Logged
public class Hood extends SubsystemBase {
	private final TalonFX hoodMotor;
	/** Creates a new Hood. */
	private final MotionMagicVoltage positionMotionMagic = new MotionMagicVoltage(0);
	private final CANcoder hoodEncoder = new CANcoder(HoodConstants.HOOT_ENCODER_CAN_ID);
	private NetworkTable hoodTable;
	private DoubleEntry hoodPositionEntry;
	private Angle requestedAngle;

	public Hood(NetworkTable table) {
		setupNetworkTable(table);
		hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_CAN_ID);
		requestedAngle = Degrees.zero();
		TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

		hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		// Configure the motor to make sure positive voltage is counter clockwise
		hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		hoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Use cosine gravity compensation
		hoodConfig.Slot0.kG = HoodConstants.KG; // Gravity gain
		hoodConfig.Slot0.kS = HoodConstants.KS; // Static gain
		hoodConfig.Slot0.kP = HoodConstants.KP; // Proportional gain
		hoodConfig.Slot0.kD = HoodConstants.KD; // Derivative gain
		hoodConfig.Slot0.kV = HoodConstants.KV;
		hoodConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
		hoodConfig.CurrentLimits.SupplyCurrentLowerLimit = HoodConstants.SUPPLY_CURRENT_LOWER_LIMIT; // Limit to 40A
		hoodConfig.CurrentLimits.SupplyCurrentLimit = HoodConstants.SUPPLY_CURRENT_LIMIT;
		hoodConfig.CurrentLimits.SupplyCurrentLowerTime = HoodConstants.SUPPLY_CURRENT_LOWER_TIME; // For longer than 0.1 seconds
		hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = HoodConstants.SUPPLY_CURRENT_LIMIT_ENABLE; // Enable the limit
		hoodConfig.CurrentLimits.StatorCurrentLimit = HoodConstants.STATOR_CURRENT_LIMIT; // ← Prevent motor burnout
		hoodConfig.CurrentLimits.StatorCurrentLimitEnable = HoodConstants.STATOR_CURRENT_LIMIT_ENABLE;
		hoodConfig.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.CRUISE_VELOCITY;
		hoodConfig.MotionMagic.MotionMagicAcceleration = HoodConstants.ACCELERATION;
		hoodConfig.Feedback.SensorToMechanismRatio = HoodConstants.SENSOR_TO_MECHANISM_RATIO;

		// Try to apply config multiple time. Break after successfully applying
		for (int i = 0; i < 2; ++i) {
			var status = hoodMotor.getConfigurator().apply(hoodConfig);
			if (status.isOK())
				break;
		}

		hoodMotor.setPosition(hoodEncoder.getAbsolutePosition().getValueAsDouble());

		requestedAngle = Units.Rotations.of(hoodMotor.getPosition().getValueAsDouble());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		// SmartDashboard.putBoolean("Hood at position", IsAtPosition());
	}

	public boolean IsAtPosition() {
		// System.out.println("Angle is at postition " + hoodMotor.getPosition().getValueAsDouble() + " requested " + requestedAngle.in(Degrees));
		return MathUtil.isNear(requestedAngle.in(Degrees), hoodMotor.getPosition().getValueAsDouble(), HoodConstants.TOLERANCE.in(Degrees));
	}

	public void MoveToPosition(Angle position) {
		// Apply the position output to the leader motor
		position = Degrees.of(MathUtil.clamp(position.in(Degrees), HoodConstants.MINIMUM_HOOD_ANGLE.in(Degrees), HoodConstants.MAXIMUM_HOOD_ANGLE.in(Degrees)));
		hoodMotor.setControl(positionMotionMagic.withPosition(position.in(Degrees)));
		requestedAngle = position;
		// System.out.println("Angle is at postition" + position.in(Degrees));
	}

	public Command MoveToPositionCommand(Supplier<Angle> angle) {
		return runOnce(() -> MoveToPosition(angle.get()));
	}

	private void setupNetworkTable(NetworkTable table) {
		hoodTable = table;
		hoodPositionEntry = hoodTable.getDoubleTopic(("hood angle")).getEntry(10);
		hoodPositionEntry.set(10);
	}
}