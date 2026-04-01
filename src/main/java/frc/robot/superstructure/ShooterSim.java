package frc.robot.superstructure;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperUptake;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

/**
 * Simulation functionality for the shooter.
 * <p>
 * For now, this only really works for shoot-while-move since I didn't want to do all the calculations backwards to go back to gamepiece theta and speed. But that's all we really need it for anyway.
 */
public class ShooterSim {
	private final CommandSwerveDrivetrain drivetrain;
	private final Shooter flywheel;
	private final Hood hood;
	private final Turret turret;
	private final HopperUptake hopperUptake;
	private static String TABLE_NAME = SuperstructureConstants.SHOOTER_SUPERSTRUCTURE_TABLE_NAME + "/Shooting Calculator/Sim";
	private static DoublePublisher velocityPublisher = NetworkTableInstance.getDefault()
			.getDoubleTopic(TABLE_NAME + "/INIT_VELOCITY")
			.publish();
	private static DoublePublisher anglePublisher = NetworkTableInstance.getDefault()
			.getDoubleTopic(TABLE_NAME + "/INIT_ANGLE")
			.publish();
	private static DoublePublisher flywheelSpeedPublisher = NetworkTableInstance.getDefault()
			.getDoubleTopic(TABLE_NAME + "/INIT_FLYWHEEL")
			.publish();
	private static DoublePublisher turretAnglePublisher = NetworkTableInstance.getDefault()
			.getDoubleTopic(TABLE_NAME + "/INIT_TURRET_ANGLE")
			.publish();
	/**
	 * Update counter, used for timing.
	 * <p>
	 * Incremented once per periodic call.
	 */
	private double tickCounter = 0;

	/**
	 * Stores the gamepiece speed from calculations so we can shoot at that speed.
	 */
	private static LinearVelocity gamepieceSpeed = MetersPerSecond.zero();
	/**
	 * Stores the gamepiece angle from calculations so we can shoot at that angle.
	 */
	private static Angle gamepieceTheta = Radians.zero();

	private final StructArrayPublisher<Pose3d> fuelPoses = NetworkTableInstance.getDefault()
			.getStructArrayTopic("MapleSim/Fuel Poses", Pose3d.struct)
			.publish();
	private final StructArrayPublisher<Pose3d> fuelTrajectoryHit = NetworkTableInstance.getDefault()
			.getStructArrayTopic("MapleSim/Fuel Trajectory (Hit)", Pose3d.struct)
			.publish();
	private final StructArrayPublisher<Pose3d> fuelTrajectoryMiss = NetworkTableInstance.getDefault()
			.getStructArrayTopic("MapleSim/Fuel Trajectory (Miss)", Pose3d.struct)
			.publish();

	public ShooterSim(CommandSwerveDrivetrain drivetrain, Shooter flywheel, Hood hood, Turret turret, HopperUptake hopperUptake) {
		this.drivetrain = drivetrain;
		this.flywheel = flywheel;
		this.hood = hood;
		this.turret = turret;
		this.hopperUptake = hopperUptake;
	}

	/**
	 * Periodically updates values for the shooter simulation.
	 */
	public void simulationPeriodic() {
		// Get the positions of the fuel (both on the field and in the air)
		Pose3d[] fuelPoseArray = SimulatedArena.getInstance()
				.getGamePiecesArrayByType("Fuel");
		fuelPoses.accept(fuelPoseArray);

		tickCounter += 1;
		if (tickCounter >= 20) {
			if (hopperUptake.getIsHopperRunningForwards()) {
				// If we're running the hopper, we just assume we can shoot and start simulating shooting balls

				Pose2d robotPose = drivetrain.getPose2d();
				ChassisSpeeds robotVelocity = drivetrain.getFeildRelativeSpeeds();

				ShooterValues results = ShootingCalculator.solveShootWhileMoveInterpolated(new Pose3d(robotPose), ShootingCalculator.getTargetPoseForPosition(robotPose).get(), robotVelocity);
				gamepieceSpeed = MetersPerSecond.of(results.getFlywheelSpeed().in(RotationsPerSecond) * ShootingConstants.LINREG_FLYWHEEL_A + ShootingConstants.LINREG_FLYWHEEL_B);
				Pose3d turretPose = ShootingCalculator.solveExitPose(new Pose3d(robotPose), results.getTurretAngle(), results.getHoodAngle());

				gamepieceTheta = ShootingCalculator.solveOutputAngleFromVelocity(gamepieceSpeed, ShootingCalculator.solveGamepieceTranslation(turretPose, ShootingCalculator.getTargetPoseForPosition(robotPose).get()));

				System.out.println("SHOT INFO   V:" + gamepieceSpeed.in(MetersPerSecond) + " Otheta:" + gamepieceTheta + " Ttheta:" + results.getTurretAngle().in(Radians));
				flywheelSpeedPublisher.set(results.getFlywheelSpeed().in(RotationsPerSecond));
				turretAnglePublisher.set(results.getTurretAngle().in(Radians));
				RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
						// Specify the position of the chassis
						robotPose.getTranslation(),
						// Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
						turretPose.minus(new Pose3d(robotPose)).getTranslation().toTranslation2d(),
						// Specify the field-relative speed of the chassis, adding it to the initial velocity of the projectile
						robotVelocity,
						// Turret facing direction
						new Rotation2d(results.getTurretAngle().in(Radians)), // .plus(robotPose.getRotation()),
						// Initial height of the flying note
						turretPose.getMeasureZ(),
						// The launch speed
						gamepieceSpeed,
						// The launch angle
						gamepieceTheta);
				velocityPublisher.set(gamepieceSpeed.in(MetersPerSecond));
				anglePublisher.set(gamepieceTheta.in(Degrees));
				// Configure callbacks to visualize the flight trajectory of the projectile
				fuelOnFly.withProjectileTrajectoryDisplayCallBack(
						// Callback for when the fuel will eventually hit the target (if configured)
						(pose3ds) -> fuelTrajectoryHit.accept(pose3ds.toArray(Pose3d[]::new)),
						// Callback for when the fuel will eventually miss the target, or if no target is configured
						(pose3ds) -> fuelTrajectoryMiss.accept(pose3ds.toArray(Pose3d[]::new)));

				SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
				tickCounter = 0;
			}
		}
	}

	public static void setValues(LinearVelocity gamepieceSpeed, Angle gamepieceTheta) {
		ShooterSim.gamepieceSpeed = gamepieceSpeed;
		ShooterSim.gamepieceTheta = gamepieceTheta;
	}
}
