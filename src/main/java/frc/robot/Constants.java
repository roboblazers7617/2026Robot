// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
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
		public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
	}

	public static class ShooterConstants {
		public static final int LEADER_CAN_ID = 30;
		public static final int FOLLOWER_CAN_ID = 31;
		public static final double KP = 0.15;
		public static final double KI = 0;
		public static final double KD = 0;
		public static final double KV = 0.115;
		public static final double KS = 0;
		public static final AngularVelocity RPS = Units.RotationsPerSecond.of(20);
		public static final AngularVelocity CRUISE_VELOCITY = RPS;
		public static final double ACCELERATION = 20;
		public static final boolean ENABLE_STATOR_LIMIT = true;
		public static final double STATOR_CURRENT_LIMIT = 40;
		public static final boolean ENABLE_SUPPLY_LIMIT = false;
		public static final double SUPPLY_CURRENT_LIMIT = 60.0;
		public static final AngularVelocity FAST_SPEED = CRUISE_VELOCITY;
		public static final AngularVelocity SLOW_SPEED = CRUISE_VELOCITY.div(4.0);
		public static final AngularVelocity COAST_SPEED = CRUISE_VELOCITY.div(2.0);
		public static final AngularVelocity TOLERANCE = Units.RotationsPerSecond.of(80);
		public static final double SUPPLY_CURRENT_LOWER_LIMIT = 40.0;
		public static final double SUPPLY_CURRENT_LOWER_TIME = 0.1;
		public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
	}

	public static class HoodConstants {
		public static final int HOOD_MOTOR_CAN_ID = 40;
		public static final double KP = 10;
		public static final double KS = 0;
		public static final double KD = 0;
		public static final double KV = 0.115;
		public static final double KG = 0;
		public static final Angle ANGLE = Units.Degrees.of(90);
		public static final double RPS = 20.0;
		public static final double CRUISE_VELOCITY = RPS;
		public static final double ACCELERATION = 2.0 * CRUISE_VELOCITY;
		public static final Angle TOLERANCE = Units.Degrees.of(5.0);
		public static final Angle ANGLE1 = Units.Degrees.of(180);
		public static final double SUPPLY_CURRENT_LOWER_LIMIT = 30.0;
		public static final double SUPPLY_CURRENT_LIMIT = 50.0;
		public static final double SUPPLY_CURRENT_LOWER_TIME = 0.15;
		public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
		public static final double STATOR_CURRENT_LIMIT = 60.0;
		public static final boolean STATOR_CURRENT_LIMIT_ENABLE = true;
	}
}
