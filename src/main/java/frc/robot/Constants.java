// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;

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
		public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
	}

	public static class HopperConstants {
		// Can ID and DIO pins
		// UPTAKE MOTOR
		public static final int BIG_SPINNY_CAN_ID = 0;
		// HOPPER MOTOR
		public static final int LITTLE_SPINNY_CAN_ID = 2;
		public static final int BEAM_BREAK_DIO_PIN = 0;
		// UPTAKE PID values
		public static final double UPTAKE_KP = 0.15;
		public static final double UPTAKE_KI = 0;
		public static final double UPTAKE_KD = 0;
		public static final double UPTAKE_KV = 0.115;
		public static final double UPTAKE_KS = 0;
		// UPTAKE Config setup
		public static final int UPTAKE_STATOR_CURRENT_LIMIT = 40;
		public static final double UPTAKE_SUPPLY_CURRENT_LIMIT = 60;
		public static final int UPTAKE_LOWER_CURRENT_LIMIT = 40;
		public static final boolean UPTAKE_ENABLE_STATOR_LIMIT = true;
		public static final boolean UPTAKE_ENABLE_SUPPLY_LIMIT = false;
		// HOPPER PID values
		public static final double HOPPER_KP = 0.15;
		public static final double HOPPER_KI = 0;
		public static final double HOPPER_KD = 0;
		public static final double HOPPER_KV = 0.115;
		public static final double HOPPER_KS = 0;
		// HOPPER Config setup
		public static final int HOPPER_STATOR_CURRENT_LIMIT = 40;
		public static final double HOPPER_SUPPLY_CURRENT_LIMIT = 50;
		public static final int HOPPER_LOWER_CURRENT_LIMIT = 40;
		public static final boolean HOPPER_ENABLE_STATOR_LIMIT = true;
		public static final boolean HOPPER_ENABLE_SUPPLY_LIMIT = false;
		// Speed Values in RPM
		public static final double FORWARD_HOPPER_RPM = 10;
		public static final double FORWARD_UPTAKE_RPM = 15;
		public static final double BACKWARD_HOPPER_RPM = -15;
		public static final double BACKWARD_UPTAKE_RPM = -15;
	}
}
