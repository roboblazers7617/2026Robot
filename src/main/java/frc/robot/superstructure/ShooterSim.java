package frc.robot.superstructure;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StubbedFlywheel;
import frc.robot.subsystems.StubbedHood;
import frc.robot.subsystems.StubbedHopperUptake;
import frc.robot.subsystems.StubbedTurret;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

/**
 * Simulation functionality for the shooter.
 * <p>
 * For now, this only really works for shoot-while-move since I didn't want to do all the calculations backwards to go back to gamepiece theta and speed. But that's all we really need it for anyway.
 */
public class ShooterSim {
	private final CommandSwerveDrivetrain drivetrain;
	private final StubbedFlywheel flywheel;
	private final StubbedHood hood;
	private final StubbedTurret turret;
	private final StubbedHopperUptake hopperUptake;

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

	public ShooterSim(CommandSwerveDrivetrain drivetrain, StubbedFlywheel flywheel, StubbedHood hood, StubbedTurret turret, StubbedHopperUptake hopperUptake) {
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
		if (tickCounter >= 10) {
			if (hopperUptake.isHopperRunning()) {
				// If we're running the hopper, we just assume we can shoot and start simulating shooting balls

				Pose2d robotPose = drivetrain.getPose2d();
				ChassisSpeeds robotVelocity = drivetrain.getFeildRelativeSpeeds();

				RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
						// Specify the position of the chassis
						robotPose.getTranslation(),
						// Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
						TurretConstants.TURRET_OFFSET.getTranslation().toTranslation2d(),
						// Specify the field-relative speed of the chassis, adding it to the initial velocity of the projectile
						robotVelocity,
						// Turret facing direction
						new Rotation2d(turret.getPosition()),
						// Initial height of the flying note
						TurretConstants.TURRET_OFFSET.getMeasureZ(),
						// The launch speed
						gamepieceSpeed,
						// The launch angle
						gamepieceTheta);

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
