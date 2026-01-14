package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.generated.VersionConstantsGenerated;

/**
 * Constants that contain version metadata for the robot. Wraps an auto-generated class to add Javadoc and logging.
 */
public class VersionConstants {
	/**
	 * Maven group which the robot package belongs to.
	 */
	public static final String MAVEN_GROUP = VersionConstantsGenerated.MAVEN_GROUP;

	/**
	 * Maven package name of the robot package.
	 */
	public static final String MAVEN_NAME = VersionConstantsGenerated.MAVEN_NAME;

	/**
	 * Robot code version.
	 */
	public static final String VERSION = VersionConstantsGenerated.VERSION;

	/**
	 * Git revision number.
	 */
	public static final int GIT_REVISION = VersionConstantsGenerated.GIT_REVISION;

	/**
	 * Git commit hash.
	 */
	public static final String GIT_SHA = VersionConstantsGenerated.GIT_SHA;

	/**
	 * Git commit date.
	 */
	public static final String GIT_DATE = VersionConstantsGenerated.GIT_DATE;

	/**
	 * Git branch.
	 */
	public static final String GIT_BRANCH = VersionConstantsGenerated.GIT_BRANCH;

	/**
	 * Date on which the robot program was built in a human-readable format.
	 */
	public static final String BUILD_DATE = VersionConstantsGenerated.BUILD_DATE;

	/**
	 * Date on which the robot program was built in unix time.
	 */
	public static final long BUILD_UNIX_TIME = VersionConstantsGenerated.BUILD_UNIX_TIME;

	/**
	 * Are there uncommited changes?
	 */
	public static final int DIRTY = VersionConstantsGenerated.DIRTY;

	/**
	 * Publish version metadata to a NetworkTable.
	 *
	 * @param table
	 *            NetworkTable to publish metadata to.
	 */
	@SuppressWarnings("all")
	public static void publishNetworkTables(NetworkTable table) {
		StringPublisher mavenGroupPublisher = table.getStringTopic("/MavenGroup")
				.publish();
		mavenGroupPublisher.set(MAVEN_GROUP);

		StringPublisher mavenNamePublisher = table.getStringTopic("/MavenName")
				.publish();
		mavenNamePublisher.set(MAVEN_NAME);

		StringPublisher versionPublisher = table.getStringTopic("/Version")
				.publish();
		versionPublisher.set(VERSION);

		StringPublisher gitRevisionPublisher = table.getStringTopic("/GitRevision")
				.publish();
		gitRevisionPublisher.set(String.valueOf(GIT_REVISION));

		StringPublisher gitShaPublisher = table.getStringTopic("/GitSHA")
				.publish();
		gitShaPublisher.set(GIT_SHA);

		StringPublisher gitDatePublisher = table.getStringTopic("/GitDate")
				.publish();
		gitDatePublisher.set(GIT_DATE);

		StringPublisher gitBranchPublisher = table.getStringTopic("/GitBranch")
				.publish();
		gitBranchPublisher.set(GIT_BRANCH);

		StringPublisher buildDatePublisher = table.getStringTopic("/BuildDate")
				.publish();
		buildDatePublisher.set(BUILD_DATE);

		StringPublisher dirtyPublisher = table.getStringTopic("/Dirty")
				.publish();
		dirtyPublisher.set((DIRTY != 0) ? "Uncommited changes" : "All changes commited");
	}

	/**
	 * Logs the metadata to SignalLogger.
	 */
	@SuppressWarnings("all")
	public static void logSignals() {
		SignalLogger.writeString("/Metadata/MavenGroup", MAVEN_GROUP);
		SignalLogger.writeString("/Metadata/MavenName", MAVEN_NAME);
		SignalLogger.writeString("/Metadata/Version", VERSION);
		SignalLogger.writeString("/Metadata/GitRevision", String.valueOf(GIT_REVISION));
		SignalLogger.writeString("/Metadata/GitSHA", GIT_SHA);
		SignalLogger.writeString("/Metadata/GitDate", GIT_DATE);
		SignalLogger.writeString("/Metadata/GitBranch", GIT_BRANCH);
		SignalLogger.writeString("/Metadata/BuildDate", BUILD_DATE);
		SignalLogger.writeString("/Metadata/Dirty", (DIRTY != 0) ? "Uncommited changes" : "All changes commited");
	}
}
