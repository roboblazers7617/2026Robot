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

	/**
	 * Constants used to configure the turret.
	 */
	public static class TurretConstants {
		/**
		 * The CAN ID for the turret motor.
		 */
		public static final int MOTOR_ID = 0;
		/**
		 * The CAN ID for the primary turret encoder.
		 */
		public static final int PRIMARY_ENCODER_ID = 0;
		/**
		 * The CAN ID for the secondary turret encoder.
		 */
		public static final int SECONDARY_ENCODER_ID = 0;

		/**
		 * The number of encoder rotations per mechanism rotation for the primary encoder.
		 */
		public static final double PRIMARY_ENCODER_RATIO = 1.0 / 1.0;
		/**
		 * The number of encoder rotations per mechanism rotation for the secondary encoder.
		 */
		public static final double SECONDARY_ENCODER_RATIO = 1.0 / 2.0;

		/**
		 * The current limit for the motor.
		 */
		public static final double MOTOR_CURRENT_LIMIT = 40.0;
		/**
		 * The PID kP for the turret closed loop controller.
		 */
		public static final double TURRET_KP = 1.0;
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
	}
}
