package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase {
	private final TalonFX elevatorLeaderMotor;
	private final TalonFX followerElevatorMotor;

	private final TalonFX topClimbHookMotor;
	private final TalonFX bottomClimbHookMotor;

	private final CANcoder topHookEncoder;
	private final CANcoder bottomHookEncoder;

	private final MotionMagicVoltage elevatorPositionOut = new MotionMagicVoltage(0);
	private final MotionMagicVoltage topHookPositionOut = new MotionMagicVoltage(0);
	private final MotionMagicVoltage bottomHookPositionOut = new MotionMagicVoltage(0);

	public Climber() {
		topHookEncoder = new CANcoder(ClimbConstants.TOP_HOOK_ENCODER_CAN_ID);
		bottomHookEncoder = new CANcoder(ClimbConstants.BOTTOM_HOOK_ENCODER_CAN_ID);

		elevatorLeaderMotor = new TalonFX(ClimbConstants.ELEVATOR_LEADER_MOTOR_CAN_ID);
		followerElevatorMotor = new TalonFX(ClimbConstants.ELEVATOR_FOLLOWER_MOTOR_CAN_ID);
		topClimbHookMotor = new TalonFX(ClimbConstants.TOP_HOOK_MOTOR_CAN_ID);
		bottomClimbHookMotor = new TalonFX(ClimbConstants.BOTTOM_HOOK_MOTOR_CAN_ID);

		followerElevatorMotor.setControl(new Follower(elevatorLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));

		TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
		elevatorConfig.Slot0.kS = ClimbConstants.KS;
		elevatorConfig.Slot0.kV = ClimbConstants.KV;
		elevatorConfig.Slot0.kP = ClimbConstants.KP;
		elevatorConfig.Slot0.kP = ClimbConstants.KI;
		elevatorConfig.Slot0.kP = ClimbConstants.KD;
		elevatorConfig.Slot0.kG = ClimbConstants.KG;
		elevatorConfig.Slot0.kA = ClimbConstants.KA;

		elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.MAX_VELOCITY;
		elevatorConfig.MotionMagic.MotionMagicAcceleration = ClimbConstants.MAX_ACCELERATION;

		TalonFXConfiguration hookMotorConfig = new TalonFXConfiguration();
		hookMotorConfig.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.MAX_HOOK_VELOCITY;
		hookMotorConfig.MotionMagic.MotionMagicAcceleration = ClimbConstants.MAX_HOOK_ACCELERATION;

		/*
		 * Apply configs multiple times bc Caleb says it breaks sometimes
		 */
		for (int i = 0; i < 2; ++i) {
			var status = elevatorLeaderMotor.getConfigurator().apply(elevatorConfig);
			var status2 = followerElevatorMotor.getConfigurator().apply(elevatorConfig);
			var status3 = topClimbHookMotor.getConfigurator().apply(hookMotorConfig);
			var status4 = bottomClimbHookMotor.getConfigurator().apply(hookMotorConfig);
			if (status.isOK() && status2.isOK() && status3.isOK() && status4.isOK())
				break;
		}
	}

	/*
	 * pos
	 */
	private void moveTo(double position) {
		if (position > ClimbConstants.MAX_POSITION) {
			throw new Error("position given " + position + " is more than max elevator height of " + ClimbConstants.MAX_POSITION);
		} else if (position < ClimbConstants.MIN_POSITION) {
			throw new Error("position given " + position + " is less than min elevator height of " + ClimbConstants.MIN_POSITION);
		}
		// convert position to rotations
		position /= Units.inchesToMeters(ClimbConstants.ELEVATOR_CHAIN_IN_PER_TOOTH) * ClimbConstants.ELEVATOR_CHAIN_SPROCKET_TEETH;
		position *= ClimbConstants.ELEVATOR_GEARBOX_RATIO;

		elevatorLeaderMotor.setControl(elevatorPositionOut.withPosition(position));
	}

	private void raiseTopHooks() {
		topClimbHookMotor.setControl(topHookPositionOut.withPosition(.25));
	}

	private void lowerTopHooks() {
		topClimbHookMotor.setControl(topHookPositionOut.withPosition(0));
	}

	private void raiseBottomHooks() {
		topClimbHookMotor.setControl(topHookPositionOut.withPosition(.25));
	}

	private void lowerBottomHooks() {
		topClimbHookMotor.setControl(topHookPositionOut.withPosition(0));
	}

	private void raiseClimb() {
		moveTo(ClimbConstants.MAX_POSITION);
		raiseTopHooks();
		raiseBottomHooks();
	}

	private void lowerClimb() {
		moveTo(ClimbConstants.MIN_POSITION);
	}

	@Override
	public void periodic() {
	}
}
