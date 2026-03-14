// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import yams.math.SmartMath;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static class AutoConstants {
		public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(5, 0.0, 0);
		public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(10, 0.0, 0.0);
	}

	/**
	 * Constants used to configure the operator controllers.
	 */
	public static class OperatorConstants {
		/**
		 * Controller port index where the driver controller is connected.
		 */
		public static final int DRIVER_CONTROLLER_PORT = 0;
		/**
		 * Controller port index where the operator controller is connected.
		 */
		public static final int OPERATOR_CONTROLLER_PORT = 1;
		/**
		 * Joystick deadband.
		 */
		public static final double DEADBAND = 0.1;
	}

	/**
	 * Constants used to configure logging.
	 * <p>
	 * During a competition debug mode should be false to reduce network and CPU usage. All data will still be logged it just won't be accessible until after the match.
	 * <p>
	 * During testing debug mode should be true to allow for real-time data viewing.
	 */
	public static class LoggingConstants {
		/**
		 * Send logging data to NetworkTables. Data is written to storage when set to false.
		 */
		public static final boolean DEBUG_MODE = true;
		/**
		 * Log all data above specified level.
		 */
		public static final Logged.Importance DEBUG_LEVEL = Logged.Importance.DEBUG;
	}

	/**
	 * Constants used to configure the driver dashboard.
	 */
	public static class DashboardConstants {
		/**
		 * The name of the tab used in Auto.
		 */
		public static final String AUTO_TAB_NAME = "Autonomous";
		/**
		 * The name of the tab used in Teleop.
		 */
		public static final String TELEOP_TAB_NAME = "Teleoperated";
	}

	/**
	 * Constants that describe the physical layout of the field.
	 */
	public static class FieldConstants {
		/**
		 * AprilTag Field Layout for the current game.
		 */
		public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
	}

	public static class DrivetrainConstants {
		/**
		 * Speed Multipliers
		 */

		public static final double MAX_SPEED_MULTIPLIER = 0.5;
		public static final double NORMAL_SPEED_MULTIPLIER = 0.35;
		public static final double SLOW_SPEED_MULTIPLIER = 0.25;
		// for use in speed multiplier ONLY
		public static final double MAX_SPEED_SWERVE = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
		// for use in other things that do not need to have maximum speed/speed multiplier
		public static final double MAX_SPEED_DEADBAND = 0.35 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
		// will set spinny mode turn speed
		public static final double MAX_ANGULAR_RATE_DEADBAND = 0.5 * RotationsPerSecond.of(0.75).in(RadiansPerSecond);
	}

	/**
	 * Constants used to configure the turret.
	 */
	public static class TurretConstants {
		/**
		 * The CAN ID for the turret motor.
		 */
		public static final int MOTOR_ID = 20;
		/**
		 * The CAN ID for the encoder CANdi.
		 */
		public static final int CANDI_ID = 20;

		/**
		 * The number of teeth on the main turret gear.
		 */
		public static final double TURRET_GEAR_TEETH = 200.0;
		/**
		 * The number of teeth on the gear on the turret motor.
		 */
		public static final double MOTOR_GEAR_TEETH = 20.0;
		/**
		 * The number of teeth on the encoder pinion.
		 */
		public static final double ENCODER_GEAR_TEETH = 20.0;
		/**
		 * The number of teeth on the primary encoder's gear.
		 */
		public static final double PRIMARY_ENCODER_GEAR_TEETH = 20.0;
		/**
		 * The number of teeth on the secondary encoder's gear.
		 */
		public static final double SECONDARY_ENCODER_GEAR_TEETH = 21.0;
		/**
		 * The ratio of motor turns to mechanism rotations.
		 */
		public static final double MOTOR_GEAR_RATIO = SmartMath.gearBox(MOTOR_GEAR_TEETH / TURRET_GEAR_TEETH);
		/**
		 * The ratio of encoder pinion turns to mechanism rotations.
		 */
		public static final double ENCODER_GEAR_RATIO = SmartMath.gearBox(ENCODER_GEAR_TEETH / TURRET_GEAR_TEETH);
		/**
		 * The number of primary encoder rotations per motor rotation.
		 */
		public static final double MOTOR_TO_PRIMARY_ENCODER_RATIO = SmartMath.gearBox(MOTOR_GEAR_RATIO, 1 / ENCODER_GEAR_RATIO, ENCODER_GEAR_TEETH / PRIMARY_ENCODER_GEAR_TEETH);
		/**
		 * The number of encoder rotations per mechanism rotation for the primary encoder.
		 */
		public static final double PRIMARY_ENCODER_RATIO = SmartMath.gearBox(ENCODER_GEAR_RATIO, ENCODER_GEAR_TEETH / PRIMARY_ENCODER_GEAR_TEETH);
		/**
		 * The number of encoder rotations per mechanism rotation for the secondary encoder.
		 */
		public static final double SECONDARY_ENCODER_RATIO = SmartMath.gearBox(ENCODER_GEAR_RATIO, ENCODER_GEAR_TEETH / SECONDARY_ENCODER_GEAR_TEETH);

		/**
		 * The neutral mode for the motor.
		 */
		public static final NeutralModeValue MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;
		/**
		 * The direction of the motor.
		 */
		public static final InvertedValue MOTOR_INVERTED = InvertedValue.Clockwise_Positive;

		/**
		 * The higher current limit for the motor. Helps get the motor started before switching to the {@link #MOTOR_CURRENT_LOWER_LIMIT}.
		 */
		public static final Current MOTOR_CURRENT_HIGHER_LIMIT = Amps.of(60.0);
		/**
		 * The lower current limit for the motor.
		 */
		public static final Current MOTOR_CURRENT_LOWER_LIMIT = Amps.of(40.0);
		/**
		 * The time to supply the {@link #MOTOR_CURRENT_HIGHER_LIMIT} for before switching to the {@link #MOTOR_CURRENT_LOWER_LIMIT}.
		 */
		public static final Time MOTOR_CURRENT_LOWER_TIME = Milliseconds.of(100);
		/**
		 * The stator current limit of the motor. This limits the current to the windings, which should help prevent burnout.
		 */
		public static final Current MOTOR_STATOR_CURRENT_LIMIT = Amps.of(60.0);

		/**
		 * The PID kP for the turret closed loop controller.
		 */
		public static final double TURRET_KP = 10.0;
		/**
		 * The PID kI for the turret closed loop controller.
		 */
		public static final double TURRET_KI = 0;
		/**
		 * The PID kD for the turret closed loop controller.
		 */
		public static final double TURRET_KD = 0;
		/**
		 * The PID kS for the turret closed loop controller.
		 */
		public static final double TURRET_KS = 0.25;
		/**
		 * The PID kV for the turret closed loop controller.
		 */
		public static final double TURRET_KV = 0.12;
		/**
		 * The PID kA for the turret closed loop controller.
		 */
		public static final double TURRET_KA = 0.01;
		/**
		 * The peak/cruising velocity of the motion.
		 */
		public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(80.0);
		/**
		 * The acceleration and deceleration rates during the beginning and end of motion.
		 */
		public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(160.0);
		/**
		 * Jerk (derivative of acceleration).
		 */
		public static final Velocity<AngularAccelerationUnit> JERK = RotationsPerSecondPerSecond.of(1600).per(Second);

		/**
		 * The lowest angle the turret can rotate to.
		 */
		public static final Angle MINIMUM_ANGLE = Rotations.of(-1.5);
		/**
		 * The highest angle the turret can rotate to.
		 */
		public static final Angle MAXIMUM_ANGLE = Rotations.of(1.5);

		/**
		 * How close does the Turret have to be to its setpoint to be counted as being there.
		 */
		public static final Measure<AngleUnit> SETPOINT_THRESHOLD = Degrees.of(3.0);
	}
}
