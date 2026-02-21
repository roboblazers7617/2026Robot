package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RunOnceDeferred;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Class which sets up driver station dashboard ? based heavily off of BC's code
 */
public class RebuiltDashboard {
	private final CommandSwerveDrivetrain commandSwerveDrivetrain;
	private final RobotContainer robotContainer;
	final SendableChooser<DriverStation.Alliance> alliancePicker;
	final SendableChooser<Pose2d> pose;
	SendableChooser<Command> auto;
	private final BooleanPublisher isEmpty;

	/**
	 * Constructor for dashboard
	 * 
	 * @param commandSwerveDrivetrain
	 * @param robotContainer
	 */
	public RebuiltDashboard(CommandSwerveDrivetrain commandSwerveDrivetrain, RobotContainer robotContainer) {
		// attaches object variables to class variables
		this.commandSwerveDrivetrain = commandSwerveDrivetrain;
		this.robotContainer = robotContainer;

		// makes alliance selection menu in gui
		alliancePicker = new SendableChooser<DriverStation.Alliance>();
		alliancePicker.setDefaultOption("None", null);
		alliancePicker.addOption("Blue", DriverStation.Alliance.Blue);
		alliancePicker.addOption("Red", DriverStation.Alliance.Red);

		alliancePicker.onChange((alliance) -> {
			if (alliance == null) {
				System.err.println("BAD! Alliance chooser run without selected alliance");
				return;
			}
			new RunOnceDeferred(() -> {
				configureAutoBuilder(alliance);
			}).ignoringDisable(true).schedule();
		});

		// makes pose selection menu in gui ?
		pose = new SendableChooser<Pose2d>();
		pose.setDefaultOption("center edge on blue side", new Pose2d(.5, 4, new Rotation2d(0)));
		pose.addOption("position 2", new Pose2d(0, 0, new Rotation2d(45)));
		pose.addOption("center edge on red side", new Pose2d(17, 4, new Rotation2d(0)));

		isEmpty = NetworkTableInstance.getDefault().getTable("").getBooleanTopic("isEmpty0").publish();
		isEmpty.set(true);
		new Trigger(() -> robotContainer.getIsBallInUptake()).onChange(Commands.runOnce(() -> isEmpty.set(robotContainer.getIsBallInUptake())));

		NetworkTableInstance.getDefault().getTable("SmartDashboard/Alliance").getEntry("selected").setString("None");
		NetworkTableInstance.getDefault().getTable("SmartDashboard/Alliance").getEntry("active").setString("None");
		SmartDashboard.putData("Alliance", alliancePicker);
		SmartDashboard.putData("Pose", pose);
		SmartDashboard.putData("Reset pose to selected position", resetPose());
	}

	/**
	 * Documentation by BC
	 * Configures the auto builder using the drivetrain subsystem. Also sets up the auto chooser on the dashboard.
	 *
	 * @param alliance
	 *            The alliance to configure the auto builder for.
	 */
	public void configureAutoBuilder(DriverStation.Alliance alliance) {
		Auto.setupPathPlanner(commandSwerveDrivetrain, alliance);
		System.out.println("Configured path planner");

		auto = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto", auto);
		robotContainer.setAutoChooser(auto);
	}

	/**
	 * I don't know what this does
	 * 
	 * @return
	 */
	private Command resetPose() {
		return new InstantCommand(() -> {
			commandSwerveDrivetrain.resetPose(pose.getSelected());
		}).ignoringDisable(true);
	}
}
