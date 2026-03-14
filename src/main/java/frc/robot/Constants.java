// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;

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

	public static class HopperConstants {
		// Can ID and DIO pins
		// UPTAKE MOTOR
		public static final int BIG_SPINNY_CAN_ID = 33;
		public static final double UPTAKE_GEAR_RATIO = 1.0;
		// HOPPER MOTOR
		public static final int LITTLE_SPINNY_CAN_ID = 32;
		public static final double HOPPER_GEAR_RATIO = 4.0;
		// UPTAKE PID values
		public static final double UPTAKE_KP = 0.38;
		public static final double UPTAKE_KI = 0;
		public static final double UPTAKE_KD = 0;
		public static final double UPTAKE_KV = 0.12;
		public static final double UPTAKE_KS = 0.29;
		// UPTAKE Config setup
		public static final int UPTAKE_STATOR_CURRENT_LIMIT = 60;
		public static final double UPTAKE_SUPPLY_CURRENT_LIMIT = 70;
		public static final int UPTAKE_LOWER_CURRENT_LIMIT = 40;
		public static final boolean UPTAKE_ENABLE_STATOR_LIMIT = true;
		public static final boolean UPTAKE_ENABLE_SUPPLY_LIMIT = false;
		public static final InvertedValue UPTAKE_IS_INVERTED = InvertedValue.CounterClockwise_Positive;
		public static final double UPTAKE_MAXIMUM_VELOCITY = (100 / UPTAKE_GEAR_RATIO) * 0.8;
		public static final double UPTAKE_ACCELERATION = UPTAKE_MAXIMUM_VELOCITY * 2.0;
		// HOPPER PID values
		public static final double HOPPER_KP = 0.22;
		public static final double HOPPER_KI = 0;
		public static final double HOPPER_KD = 0.01;
		public static final double HOPPER_KV = 0.115;
		public static final double HOPPER_KS = 0.25;
		// HOPPER Config setup
		public static final int HOPPER_STATOR_CURRENT_LIMIT = 40;
		public static final double HOPPER_SUPPLY_CURRENT_LIMIT = 70;
		public static final int HOPPER_LOWER_CURRENT_LIMIT = 40;
		public static final boolean HOPPER_ENABLE_STATOR_LIMIT = true;
		public static final boolean HOPPER_ENABLE_SUPPLY_LIMIT = false;
		public static final InvertedValue HOPPER_IS_INVERTED = InvertedValue.Clockwise_Positive;
		public static final double HOPPER_MAXIMUM_VELOCITY = (100 / HOPPER_GEAR_RATIO) * 0.8;
		public static final double HOPPER_ACCELERATION = HOPPER_MAXIMUM_VELOCITY * 2.0;
		// Speed Values in RPM
		public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(5);
		public static final AngularVelocity FORWARD_HOPPER_RPS = RotationsPerSecond.of(10.0);
		public static final AngularVelocity FORWARD_UPTAKE_RPS = RotationsPerSecond.of(35.0);
		public static final AngularVelocity BACKWARD_HOPPER_RPS = RotationsPerSecond.of(-10.0);
		public static final AngularVelocity BACKWARD_UPTAKE_RPS = RotationsPerSecond.of(-15.0);
	}

	public static class ShooterConstants {
		public static final int LEADER_CAN_ID = 21;
		public static final int FOLLOWER_CAN_ID = 22;
		public static final double KP_0 = 0.07;
		public static final double KI_0 = 0;
		public static final double KD_0 = 0;
		public static final double KV_0 = 0.126;
		public static final double KS_0 = 0.45;
		public static final double KP_1 = 0.07;
		public static final double KI_1 = 0;
		public static final double KD_1 = 0;
		public static final double KV_1 = 0.126;
		public static final double KS_1 = 0.45;
		public static final AngularVelocity RPS = Units.RotationsPerSecond.of(50);
		public static final AngularVelocity CRUISE_VELOCITY = RPS;
		public static final double MAXIMUM_VELOCITY = 80.0;
		public static final double ACCELERATION = 2.0 * MAXIMUM_VELOCITY;
		public static final boolean ENABLE_STATOR_LIMIT = true;
		public static final double STATOR_CURRENT_LIMIT = 40;
		public static final boolean ENABLE_SUPPLY_LIMIT = false;
		public static final double SUPPLY_CURRENT_LIMIT = 60.0;
		public static final AngularVelocity FAST_SPEED = CRUISE_VELOCITY;
		public static final AngularVelocity SLOW_SPEED = CRUISE_VELOCITY.div(3.0);
		public static final AngularVelocity COAST_SPEED = CRUISE_VELOCITY.div(2.0);
		public static final AngularVelocity TOLERANCE = Units.RotationsPerSecond.of(3);
		public static final double SUPPLY_CURRENT_LOWER_LIMIT = 40.0;
		public static final double SUPPLY_CURRENT_LOWER_TIME = 0.1;
		public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
	}

	public static class HoodConstants {
		public static final int HOOD_MOTOR_CAN_ID = 23;
		public static final int HOOT_ENCODER_CAN_ID = 23;
		public static final double KP = 0.3;
		public static final double KS = 0.4;
		public static final double KD = 0;
		public static final double KV = 0.03;
		public static final double KG = 0.6;
		public static final Angle ANGLE = Units.Degrees.of(90);
		public static final double RPS = 100.0; // X44 can go 125 Rotations per Second
		public static final double CRUISE_VELOCITY = RPS;
		public static final double ACCELERATION = 3.0 * CRUISE_VELOCITY;
		public static final Angle TOLERANCE = Units.Degrees.of(1.0);
		public static final double SUPPLY_CURRENT_LOWER_LIMIT = 30.0;
		public static final double SUPPLY_CURRENT_LIMIT = 50.0;
		public static final double SUPPLY_CURRENT_LOWER_TIME = 0.15;
		public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
		public static final double STATOR_CURRENT_LIMIT = 60.0;
		public static final boolean STATOR_CURRENT_LIMIT_ENABLE = true;
		public static final Angle MINIMUM_HOOD_ANGLE = Degrees.of(1);
		public static final Angle MAXIMUM_HOOD_ANGLE = Degrees.of(32);
		public static final double SENSOR_TO_MECHANISM_RATIO = 13.78 / 32; // Number of shaft rotations / degrees traveled
	}
}
