package frc.robot.utilities;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.DriveConstants;

/**
 * Trajectory methods for going to the trench from the center position
 */
public class TrajectoryTrenchFromCenter {

	/**
	* Calculate the trajectory used to get the balls from the trench starting from the center
	* (the center is directly in front of the target)
	*/
	public static Trajectory calcTrajectory(FileLog log) {
		Trajectory trajectory = null;
		DifferentialDriveKinematics driveKinematics = TrajectoryUtil.getDriveKinematics();
	
    	try {

			log.writeLogEcho(true, "AutoTrench", "Trajectory", 
				"trackWidth",DriveConstants.TRACK_WIDTH,
				"maxVoltage", DriveConstants.MAX_VOLTAGE, 
				"kS", DriveConstants.kS, 
				"kV", DriveConstants.kV, 
				"kA", DriveConstants.kA,
				"kP", DriveConstants.kP,
				"maxSpeed", DriveConstants.kMaxSpeedMetersPerSecond,
				"maxAcceleration", DriveConstants.kMaxAccelerationMetersPerSecondSquared);

			// Create a voltage constraint to ensure we don't accelerate too fast
			DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA), 
				driveKinematics,
				DriveConstants.MAX_VOLTAGE);

			// Create config for trajectory
			TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
				DriveConstants.kMaxAccelerationMetersPerSecondSquared)
				.setKinematics(driveKinematics)
				.addConstraint(autoVoltageConstraint);

			// drive from line to trench (assumes starting directly in front and facing target)		  
			// the trajectory to follow (all units in meters)
			// start at the origin facing the +X direction
			// Y is distance forward
			// firstBallY = -1.4
			// firstBallX = -3.1
			// distance between first and last ball = 1.82
			trajectory = TrajectoryGenerator.generateTrajectory(
				new Pose2d(0, 0, new Rotation2d(0.0)),
				List.of(
					//new Translation2d(-0.5, -0.5),
					new Translation2d(1, 0) // actual is -1.4 but move over to make sure we miss the wall
				),
				new Pose2d(2, 0, new Rotation2d(0.0)), config);

			// debug logging
			TrajectoryUtil.dumpTrajectory(trajectory, log);

		} catch (Exception e) {
			log.writeLogEcho(true, "AutoTrench", "Trajectory", 
				"ERROR in calcTrajectory", e.toString(),"exception",e);
		}

		if (trajectory != null) {
			log.writeLogEcho(true, "AutoTrench", "Trajectory", "SUCCESS", true);
		};
	
		return trajectory;
	}

	

}
