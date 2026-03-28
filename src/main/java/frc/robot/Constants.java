// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import frc.robot.generated.TunerConstants;
import frc.robot.superstructure.ShooterValues;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.BiAlliancePose3d;
import frc.robot.util.InterpolatingMeasureTreeMap;
import frc.robot.util.PoseUtil;
import frc.robot.util.RectangleUtil;

import edu.wpi.first.units.Units;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import yams.math.SmartMath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	/**
	 * The CAN bus used by devices on the CANivore.
	 */
	public static final CANBus CANIVORE_BUS = new CANBus("CANivore");

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
	 * During a competition debug mode should be false to reduce network and CPU
	 * usage. All data will still be logged it just won't be accessible until after
	 * the match.
	 * <p>
	 * During testing debug mode should be true to allow for real-time data viewing.
	 */
	public static class LoggingConstants {
		/**
		 * Send logging data to NetworkTables. Data is written to storage when set to
		 * false.
		 */
		public static final boolean DEBUG_MODE = false;
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

		/**
		 * The temperature at which a warning is shown on the dashboard about motor overtemp.
		 */
		public static final Temperature MOTOR_WARNING_TEMPERATURE = Celsius.of(50.0);
		/**
		 * The temperature the motor has to fall to for another alert to be sent. This adds a bit of hysteresis.
		 */
		public static final Temperature MOTOR_WARNING_RESET_TEMPERATURE = MOTOR_WARNING_TEMPERATURE.minus(Celsius.of(2.0));
		/**
		 * How long to wait between each update of the motor warning class.
		 */
		public static final Time MOTOR_WARNING_INTERVAL = Seconds.of(1.0);
	}

	/**
	 * Constants that describe the physical layout of the field.
	 */
	public static class FieldConstants {
		/**
		 * AprilTag Field Layout for the current game.
		 */
		public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
		/**
		 * A pose at the center of the field.
		 */
		public static final Pose2d FIELD_CENTER = new Pose2d(FIELD_LAYOUT.getFieldLength() / 2, FIELD_LAYOUT.getFieldWidth() / 2, Rotation2d.kZero);
		/**
		 * A rectangle encompassing the neutral (center) zone of the field.
		 */
		public static final Rectangle2d NEUTRAL_RECTANGLE = new Rectangle2d(FIELD_CENTER, Inches.of(287.0), Meters.of(FIELD_LAYOUT.getFieldWidth()));

		/**
		 * Zones and poses to shoot from and to.
		 */
		public static class ShootingZones {
			/**
			 * A rectangle encompassing the shooting zone for the top half (towards negative Y) of the neutral rectangle (when looking at the field diagram).
			 */
			public static final Rectangle2d NEUTRAL_ZONE_TOP = new Rectangle2d(FIELD_CENTER.transformBy(new Transform2d(0, -FIELD_LAYOUT.getFieldWidth() / 3.0, Rotation2d.kZero)), NEUTRAL_RECTANGLE.getXWidth(), NEUTRAL_RECTANGLE.getYWidth() / 3.0);
			/**
			 * A rectangle encompassing the shooting zone for the bottom half (towards positive Y) of the neutral rectangle (when looking at the field diagram).
			 */
			public static final Rectangle2d NEUTRAL_ZONE_BOTTOM = RectangleUtil.flipRectangleY(NEUTRAL_ZONE_TOP);
			/**
			 * The pose to shoot at for the {@link #NEUTRAL_ZONE_TOP top of the neutral rectangle} (when looking at the field diagram).
			 */
			public static final BiAlliancePose3d SHUTTLE_TOP_POSE = BiAlliancePose3d.fromBluePose(new Pose3d(Meters.of(3.0), Meters.of(2.0), Meters.zero(), Rotation3d.kZero), BiAlliancePose3d.InvertY.KEEP_Y);
			/**
			 * The pose to shoot at for the {@link #NEUTRAL_ZONE_BOTTOM bottom of the neutral rectangle} (when looking at the field diagram).
			 */
			public static final BiAlliancePose3d SHUTTLE_BOTTOM_POSE = BiAlliancePose3d.fromBluePose(PoseUtil.flipPoseY(SHUTTLE_TOP_POSE.getBluePose()), BiAlliancePose3d.InvertY.KEEP_Y);

			/**
			 * A rectangle encompassing the shooting zone for the hub on the red alliance.
			 */
			public static final Rectangle2d HUB_ZONE_RED = new Rectangle2d(FIELD_CENTER.transformBy(new Transform2d(Inches.of(234.555).plus(Meters.of(0.1)), Meters.zero(), Rotation2d.kZero)), Inches.of(182.11), Inches.of(317.7));
			/**
			 * A rectangle encompassing the shooting zone for the hub on the blue alliance.
			 */
			public static final Rectangle2d HUB_ZONE_BLUE = RectangleUtil.flipRectangleX(HUB_ZONE_RED);
			/**
			 * The pose to shoot at for the {@link #HUB_ZONE_RED} and {@link #HUB_ZONE_BLUE}.
			 */
			public static final BiAlliancePose3d HUB_POSE = BiAlliancePose3d.fromRedPose(new Pose3d(FIELD_CENTER).transformBy(new Transform3d(Inches.of(143.50), Meters.zero(), Inches.of(72.0), Rotation3d.kZero)), BiAlliancePose3d.InvertY.KEEP_Y);
		}
	}

	/**
	 * This is MY intake constant class which I balcantara made btw I don't know
	 * what it does YET (it contains constants for my dogwater code for intake)
	 */
	public static class IntakeConstants {
		public static final int SHOULDER_CAN_ID = 40;
		public static final int GRABBER_CAN_ID = 41;
		public static final int SHOULDER_ENCODER_CAN_ID = 40;

		/**
		 * The current limit for the motor.
		 */
		public static final double GRABBER_SUPPLY_CURRENT_LIMIT = 30.0;
		// public static final double INTAKE_START_SPEED = 0.2; //ts is old (as are the
		// following three)
		public static final double INTAKE_START_VOLTAGE = 6; // 4.5
		public static final double INTAKE_START_SLOW_VOLTAGE = 3.0; // 2.0
		// public static final double INTAKE_STOP_SPEED = 0.0;
		public static final double INTAKE_STOP_VOLTAGE = 0.0;
		// public static final double OUTTAKE_SPEED = -0.2;
		public static final double OUTTAKE_VOLTAGE = -2;

		// intake in-out position constants
		// public static final Angle SHOULDER_STOWED_ANGLE = Degrees.of(90);
		// public static final Angle SHOULDER_STOWED_ANGLE = Rotations.of(0);
		public static final double SHOULDER_MINIMUM_DISTANCE = 0;
		// public static final Angle SHOULDER_LOWERED_ANGLE = Degrees.of(0);
		// public static final Angle SHOULDER_LOWERED_ANGLE = Rotations.of(0);
		public static final double SHOULDER_MAXIMUM_DISTANCE = 36.5;
		// public static final Angle SHOULDER_DEPOT_ANGLE = Degrees.of(10);
		// public static final Angle SHOULDER_DEPOT_ANGLE = Rotations.of(0);
		public static final double SHOULDER_DEPOT_DISTANCE = 0.25;
		public static final double SHOULDER_STOW_OVER_BUMPER_DISTANCE = 10.0;

		// PID vals for moving the intake in
		public static final double INTAKE_KG_0 = 0.1;
		public static final double INTAKE_KS_0 = 0.34;
		public static final double INTAKE_KV_0 = 0.16;
		public static final double INTAKE_KA_0 = 0;
		public static final double INTAKE_KP_0 = 0.2;
		public static final double INTAKE_KI_0 = 0;
		public static final double INTAKE_KD_0 = 0;

		// PID vals for moving the intake out
		public static final double INTAKE_KG_1 = 0.1;
		public static final double INTAKE_KS_1 = 0.34;
		public static final double INTAKE_KV_1 = 0.13;
		public static final double INTAKE_KA_1 = 0;
		public static final double INTAKE_KP_1 = 0.05;
		public static final double INTAKE_KI_1 = 0;
		public static final double INTAKE_KD_1 = 0;

		// public static final double INTAKE_MM_CRUISE_VELOCITY = 80; // depracated
		// public static final double INTAKE_MM_ACCELERATION = 160; // depracated
		// public static final double INTAKE_MM_JERK = 1600; // depracated

		// new (?) system for gear ratios
		public static final double GEARBOX_RATIO = 96.0 / 5.0; // to be replaced by talonfx (?)
		// config things
		public static final double FAST_MAXIMUM_VELOCITY = 75.0;
		public static final double FAST_ACCELERATION = 4.0 * FAST_MAXIMUM_VELOCITY;

		public static final double SLOW_MAXIMUM_VELOCITY = 20.0;
		public static final double SLOW_ACCELERATION = 2.0 * FAST_MAXIMUM_VELOCITY;

		// talonfx ratio stuff
		// public static final double ROTOR_TO_SENSOR_RATIO = 0; // depracated (?)
		public static final double SENSOR_TO_MECHANISM_RATIO = 1.0; // number of shaft rotations divided by
																	// distance traveled by intake

		public static final Angle AGITATE_RAISED_ANGLE = Degrees.of(70); // depracated
		public static final Angle AGITATE_LOWERED_ANGLE = Degrees.of(0); // depracated
		public static final Angle AGITATE_TOLERANCE = Degrees.of(10);
		public static final double GRABBER_SUPPLY_CURRENT_LOWER_LIMIT = 20.0;
		public static final double GRABBER_SUPPLY_CURRENT_LOWER_TIME = 0.1;
		public static final double GRABBER_STATOR_CURRENT_LIMIT = 40.0;
		public static final double SHOULDER_SUPPLY_CURRENT_LIMIT = 40.0;
		public static final double SHOULDER_SUPPLY_CURRENT_LOWER_LIMIT = 20.0;
		public static final double SHOULDER_SUPPLY_CURRENT_LOWER_TIME = 0.15;
		public static final double SHOULDER_STATOR_CURRENT_LIMIT = 20.0;

		public static final double GAIN_SCHEDULE_ERROR_THRESHOLD = 0.5;

		public static final double SHOULDER_TOLERANCE = 0.1;
		public static final double NUDGE_SPEED = 0.3;
	}

	public static class DrivetrainConstants {
		/**
		 * Swerve request field centric facing angle constants
		 */
		public static final double HEADING_kP = 5.0;
		public static final double HEADING_ki = 0.0;
		public static final double HEADING_kd = 0.1;
		/**
		 * Swerve request general constants (for both )
		 */
		public static final double DRIVE_DEADBAND = 0.1;
		public static final double ROTATIONAL_DEADBAND = 0.1;
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
	 * Constants that configure the superstructure.
	 */
	public static class SuperstructureConstants {
		/**
		 * The table name for the shooter superstructure.
		 */
		public static final String SHOOTER_SUPERSTRUCTURE_TABLE_NAME = "Shooter Superstructure";
		/**
		 * The table name for the intake superstructure.
		 */
		public static final String INTAKE_SUPERSTRUCTURE_TABLE_NAME = "Intake Superstructure";
		/**
		 * The interval to run the {@link RobotContainer#superstructurePeriodic()} at.
		 */
		public static final Time PERIODIC_INTERVAL = Milliseconds.of(10);
		/**
		 * The number of times to recalculate the shooting position for shoot-while-move. More iterations should give more accurate shoot-while-move outputs.
		 */
		public static final int SHOOTING_CALCULATOR_ITERATIONS = 3;
		/**
		 * Should we automatically enter the {@link frc.robot.superstructure.ShooterSuperstructure.ShooterState#HOME} state on enable?
		 * <p>
		 * This should be set to true for competitions, since we want to spin up the shooter automatically.
		 */
		public static final boolean HOME_ON_ENABLE = true;

		// all things used for calculating the turret pose because if i left them as magic number Max would pipe bomb my mailbox

		/**
		 * the distance from the pivot of the hood to the halfway point between the edges of the flywheels, AKA where the ball will be launched from
		 */
		public static final Distance HOOD_PIVOT_TO_GAMEPIECE_LAUNCH_RADIUS = Meters.of(0.114351816);

		/**
		 * the transform from the center of the robot on the floor to the center of the pivot on the top of the shooter base plate, in meters
		 */
		public static final Transform3d ROBOT_TO_TURRET_BASE_TRANSFORM = new Transform3d(Inches.of(-5.25), Inches.of(-5.0625), Inches.of(14.847), new Rotation3d());
		/**
		 * the transform from the center of the pivot on top of the shooter base plate to the center of the hood pivot axis in meters
		 */
		public static final Transform3d TURRET_BASE_TO_HOOD_PIVOT = new Transform3d(0.126746, 0, 0.0635, new Rotation3d());

		/**
		 * The time to wait before overriding subsystems and just shooting anyway.
		 */
		public static final Time SHOOTING_WAITING_TIMEOUT = Seconds.of(2.0);
	}

	/**
	 * Constants that control the shooting behavior.
	 */
	public static class ShootingConstants {
		// TODO: Update all of these constants
		/**
		 * An interpolation table used for flywheel speed by gamepiece velocity.
		 */
		public static final InterpolatingMeasureTreeMap<LinearVelocity, LinearVelocityUnit, AngularVelocity, AngularVelocityUnit> FLYWHEEL_VELOCITY_BY_GAMEPIECE_VELOCITY = new InterpolatingMeasureTreeMap<>();

		static {
			// Add values to the interpolation table
			FLYWHEEL_VELOCITY_BY_GAMEPIECE_VELOCITY.put(MetersPerSecond.of(0.0), RPM.of(10.0));
			FLYWHEEL_VELOCITY_BY_GAMEPIECE_VELOCITY.put(MetersPerSecond.of(10.0), RPM.of(50.0));
		}

		/**
		 * An interpolation table used for hood angle by gamepiece velocity.
		 */
		public static final InterpolatingMeasureTreeMap<Angle, AngleUnit, Angle, AngleUnit> HOOD_ANGLE_BY_GAMEPIECE_THETA = new InterpolatingMeasureTreeMap<>();

		static {
			// Add values to the interpolation table
			HOOD_ANGLE_BY_GAMEPIECE_THETA.put(Degrees.of(37.0), Degrees.of(1.0));
			HOOD_ANGLE_BY_GAMEPIECE_THETA.put(Degrees.of(69.5), Degrees.of(32.0));
		}

		/**
		 * An interpolation table used for flywheel speed by distance, for basic shoot-from-anywhere.
		 */
		public static final InterpolatingMeasureTreeMap<Distance, DistanceUnit, AngularVelocity, AngularVelocityUnit> FLYWHEEL_VELOCITY_BY_DISTANCE = new InterpolatingMeasureTreeMap<>();

		static {
			// Add values to the interpolation table
			FLYWHEEL_VELOCITY_BY_DISTANCE.put(Meters.of(1.9), RotationsPerSecond.of(26.5));
			FLYWHEEL_VELOCITY_BY_DISTANCE.put(Meters.of(2.58), RotationsPerSecond.of(30));
			FLYWHEEL_VELOCITY_BY_DISTANCE.put(Meters.of(3.29), RotationsPerSecond.of(31.6));
			FLYWHEEL_VELOCITY_BY_DISTANCE.put(Meters.of(4.5), RotationsPerSecond.of(34.5));
			FLYWHEEL_VELOCITY_BY_DISTANCE.put(Meters.of(5.29), RotationsPerSecond.of(38));
			FLYWHEEL_VELOCITY_BY_DISTANCE.put(Meters.of(6.48), RotationsPerSecond.of(44));
		}

		/**
		 * An interpolation table used for hood angle by distance, for basic shoot-from-anywhere.
		 */
		public static final InterpolatingMeasureTreeMap<Distance, DistanceUnit, Angle, AngleUnit> HOOD_ANGLE_BY_DISTANCE = new InterpolatingMeasureTreeMap<>();

		static {
			// Add values to the interpolation table
			HOOD_ANGLE_BY_DISTANCE.put(Meters.of(1.9), Degrees.of(0));
			HOOD_ANGLE_BY_DISTANCE.put(Meters.of(2.58), Degrees.of(0));
			HOOD_ANGLE_BY_DISTANCE.put(Meters.of(3.29), Degrees.of(2));
			HOOD_ANGLE_BY_DISTANCE.put(Meters.of(4.5), Degrees.of(4));
			HOOD_ANGLE_BY_DISTANCE.put(Meters.of(5.29), Degrees.of(5));
			HOOD_ANGLE_BY_DISTANCE.put(Meters.of(6.48), Degrees.of(7));
		}

		/**
		 * The angle to shoot the gamepiece at.
		 */
		public static final Angle GAMEPIECE_THETA = Degrees.of(80.0);

		/**
		 * The acceleration due to gravity imposed on the gamepiece.
		 */
		public static final LinearAcceleration GAMEPIECE_G = MetersPerSecondPerSecond.of(-9.81);

		/**
		 * How long to wait after no balls are detected to stop shooting.
		 */
		public static final Time SHOOTING_TIMEOUT = Seconds.of(2.0);

		// TODO fill in once linreg is done
		/**
		 * the A in the linreg for x=hood angle y = output angle
		 */
		public static final double LINREG_HOOD_ANGLE_A = 0;
		/**
		 * the B in the linreg for x=hood angle y = output angle
		 */
		public static final double LINREG_HOOD_ANGLE_B = 0;

		/**
		 * the A in the linreg for x=flywheel speed y = output speed
		 */
		public static final double LINREG_FLYWHEEL_A = 0;
		/**
		 * the B in the linreg for x=flywheel speed y = output speed
		 */
		public static final double LINREG_FLYWHEEL_B = 0;

		/**
		 * The minimum angle to shoot at.
		 */
		public static final Angle MIN_SHOOT_ANGLE = Degrees.of(38);
		/**
		 * The maximum angle to shoot at.
		 */
		public static final Angle MAX_SHOOT_ANGLE = Degrees.of(69);

		/**
		 * A set of values to shoot from a static position.
		 */
		public static final ShooterValues STATIC_SHOOT_VALUES = new ShooterValues(RPM.of(2200), Degrees.of(0), Degrees.of(13));
		/**
		 * A set of values to shoot from the left by the door.
		 */
		public static final ShooterValues STATIC_SHOOT_LEFT_DOOR_VALUES = new ShooterValues(RotationsPerSecond.of(39), Degrees.of(0), Degrees.of(38));
		/**
		 * A set of values to shoot from the right by the door.
		 */
		public static final ShooterValues STATIC_SHOOT_RIGHT_DOOR_VALUES = new ShooterValues(RotationsPerSecond.of(38), Degrees.of(5), Degrees.of(-37));
		/**
		 * A set of values to shoot from the center by the tower.
		 */
		public static final ShooterValues STATIC_SHOOT_CENTER = new ShooterValues(RotationsPerSecond.of(33), Degrees.of(5), Degrees.of(0));
	}

	/**
	 * Constants used to configure the hopper.
	 */
	public static class HopperConstants {
		public static final int BEAM_BREAK_DIO_PIN = 0;
	}

	public static class HopperUptakeConstants {
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
		// // Forward
		public static final AngularVelocity FORWARD_HOPPER_RPS = RotationsPerSecond.of(15.0);
		public static final AngularVelocity FORWARD_UPTAKE_RPS = RotationsPerSecond.of(50.0);
		// // Backward
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
		public static final double MAXIMUM_VELOCITY = 80.0;
		public static final double ACCELERATION = 2.0 * MAXIMUM_VELOCITY;
		public static final boolean ENABLE_STATOR_LIMIT = true;
		public static final double STATOR_CURRENT_LIMIT = 40;
		public static final boolean ENABLE_SUPPLY_LIMIT = false;
		public static final double SUPPLY_CURRENT_LIMIT = 60.0;
		public static final AngularVelocity TOLERANCE = Units.RotationsPerSecond.of(3);
		public static final double SUPPLY_CURRENT_LOWER_LIMIT = 40.0;
		public static final double SUPPLY_CURRENT_LOWER_TIME = 0.1;
		public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;

		/**
		 * The speed for the flywheel while the shooter is homed.
		 */
		public static final AngularVelocity IDLE_SPEED = RPM.of(500.0);
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
		public static final Angle TOLERANCE = Units.Degrees.of(2.0);
		public static final double SUPPLY_CURRENT_LOWER_LIMIT = 30.0;
		public static final double SUPPLY_CURRENT_LIMIT = 50.0;
		public static final double SUPPLY_CURRENT_LOWER_TIME = 0.15;
		public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
		public static final double STATOR_CURRENT_LIMIT = 60.0;
		public static final boolean STATOR_CURRENT_LIMIT_ENABLE = true;
		public static final Angle MINIMUM_HOOD_ANGLE = Degrees.of(1);
		public static final Angle MAXIMUM_HOOD_ANGLE = Degrees.of(32);
		public static final double SENSOR_TO_MECHANISM_RATIO = 13.78 / 32; // Number of shaft rotations / degrees traveled
		public static final Angle HOME_ANGLE = MINIMUM_HOOD_ANGLE;
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
		 * The turret's offset from the center of the robot (drivetrain pose).
		 */
		public static final Transform3d TURRET_OFFSET = new Transform3d(Inches.of(5.0), Inches.of(5.062), Meters.of(0.3), Rotation3d.kZero);

		/**
		 * The number of teeth on the main turret gear.
		 */
		public static final double TURRET_GEAR_TEETH = 200.0;
		/**
		 * The number of teeth on the pinion gear, interfacing with the main turret gear.
		 */
		public static final double PINION_UPPER_TEETH = 20.0;
		/**
		 * The number of teeth on the lower pinion gear. This is 1:1 with the upper pinion gear.
		 */
		public static final double PINION_LOWER_TEETH = 24.0;
		/**
		 * The number of teeth on the gear on the turret motor and primary encoder.
		 */
		public static final double MOTOR_GEAR_TEETH = 48.0;
		/**
		 * The number of teeth on the secondary encoder's gear.
		 */
		public static final double SECONDARY_ENCODER_GEAR_TEETH = 21.0;
		/**
		 * The ratio of motor turns to mechanism rotations.
		 */
		public static final double MOTOR_GEAR_RATIO = SmartMath.gearBox(TURRET_GEAR_TEETH / PINION_UPPER_TEETH, PINION_LOWER_TEETH / MOTOR_GEAR_TEETH);
		/**
		 * The number of primary encoder rotations per motor rotation.
		 */
		public static final double MOTOR_TO_PRIMARY_ENCODER_RATIO = 1.0;
		/**
		 * The number of encoder rotations per mechanism rotation for the primary encoder.
		 */
		// public static final double PRIMARY_ENCODER_RATIO = MOTOR_GEAR_RATIO;
		public static final double PRIMARY_ENCODER_RATIO = 5 / 1;
		/**
		 * The number of encoder rotations per mechanism rotation for the secondary encoder.
		 */
		// public static final double SECONDARY_ENCODER_RATIO = SmartMath.gearBox(TURRET_GEAR_TEETH / PINION_UPPER_TEETH, PINION_UPPER_TEETH / SECONDARY_ENCODER_GEAR_TEETH);
		public static final double SECONDARY_ENCODER_RATIO = 200 / 21;

		/**
		 * the total span of the encoders tracking range, can be found by finding the lcm of the encoder ratios
		 */
		public static final double TURRET_SPAN = 4.2;

		/**
		 * the tolerance for turret encoders to consider values the same, increase if its returning null, as that means there is too much slop in the system
		 */
		public static final Angle TURRET_ENCODER_TOLERANCE = Rotations.of(.005);

		/**
		 * The offset from zero of the absolute encoder. This is in mechanism rotations.
		 */
		public static final Angle PRIMARY_ENCODER_OFFSET = Rotations.of(-0.225);
		/**
		 * The offset from zero of the absolute encoder. This is in mechanism rotations.
		 */
		public static final Angle SECONDARY_ENCODER_OFFSET = Rotations.of(0.735);

		/**
		 * The neutral mode for the motor.
		 */
		public static final NeutralModeValue MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;
		/**
		 * The direction of the motor.
		 */
		public static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;

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
		public static final double TURRET_KP = 19.0;
		/**
		 * The PID kI for the turret closed loop controller.
		 */
		public static final double TURRET_KI = 5.5;
		/**
		 * The PID kD for the turret closed loop controller.
		 */
		public static final double TURRET_KD = 2.1;
		/**
		 * The PID kS for the turret closed loop controller.
		 */
		public static final double TURRET_KS = 0.0;
		/**
		 * The PID kV for the turret closed loop controller.
		 */
		public static final double TURRET_KV = 3.0;
		/**
		 * The PID kA for the turret closed loop controller.
		 */
		public static final double TURRET_KA = 0.04;
		/**
		 * The peak/cruising velocity of the motion.
		 */
		public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(125.0).div(MOTOR_GEAR_RATIO).times(0.8);
		/**
		 * The acceleration and deceleration rates during the beginning and end of motion.
		 */
		public static final AngularAcceleration ACCELERATION = CRUISE_VELOCITY.times(2.0).per(Second);
		/**
		 * Jerk (derivative of acceleration).
		 */
		public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.times(10.0).per(Second);

		/**
		 * The lowest angle the turret can rotate to.
		 */
		public static final Angle MINIMUM_ANGLE = Rotations.of(-1.0);
		/**
		 * The highest angle the turret can rotate to.
		 */
		public static final Angle MAXIMUM_ANGLE = Rotations.of(1.0);

		/**
		 * How close does the Turret have to be to its setpoint to be counted as being there.
		 */
		public static final Measure<AngleUnit> SETPOINT_THRESHOLD = Degrees.of(6.0);
	}

	public static class VisionConstants {
		public static final Boolean IS_GO_P = true;
		public static final Boolean DEBUG_PRINT_STATEMENTS = false; // enables/disables the print statements from vision
		public static final String TURRET_CAM_NAME = "CamFront";
		public static final Transform3d ROBOT_TO_TURRET_CAM_TRANSFORM = new Transform3d(new Translation3d(-0.1397, -0.3479292, 0.2666238), new Rotation3d(0, 0.174533, -0.5 * Math.PI));
		public static final String EBOARD_CAM_NAME = "CamSide";
		public static final Transform3d ROBOT_TO_EBOARD_CAM_TRANSFORM = new Transform3d(new Translation3d(-0.3415792, -0.10795, 0.2708148), new Rotation3d(0, 0.174533, Math.PI));
		public static final String XTRA_CAM_NAME = "CamX3"; // replace the String in the Constant of the camera you want to swap out with the extra cam name
		public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8); // 4, 4, 8
		public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
	}
}
