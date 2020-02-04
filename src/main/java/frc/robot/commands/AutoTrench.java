package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;


/**
 * Auto routine starting at the initiation line and getting balls from the trench
 */
public class AutoTrench extends SequentialCommandGroup {

  // holds the trajectory we are going to follow
  // calcTrajectory() needs to be called in robotInit to populate this
  private static Trajectory trajectory;

  // holds the kinematics for this driveTrain
  private static DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH);

  /**
   * AutoTrench constructor for command group
   * @param driveTrain  The driveTrain to use to get the pose, wheel speeds and set the tankDriveVolts
   */  
  public AutoTrench(DriveTrain driveTrain, FileLog log) {

    // this should have been calculated in robotInit but check just in case
    if (trajectory == null) {
      log.writeLogEcho(true, "AutoTrench", "Trajectory", "WARNING", "AutoTrench trajectory not pre-calculated");
      calcTrajectory(log);
    }

    if (trajectory != null) {
      addCommands(
        new RamseteCommand(
          trajectory,
          driveTrain::getPose,
          new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
          new SimpleMotorFeedforward(DriveConstants.kS,DriveConstants.kV,DriveConstants.kA),
          driveKinematics,
          driveTrain::getWheelSpeeds,
          new PIDController(DriveConstants.kP, 0, 0),
          new PIDController(DriveConstants.kP, 0, 0),
          driveTrain::tankDriveVolts,
          driveTrain
        ).andThen(() -> driveTrain.tankDrive(0.0, 0.0, false))
      );
    } else {
      log.writeLogEcho(true, "AutoTrench", "Trajectory", "ERROR","null trajectory, no command has been scheduled");
    }

  }

  /**
   * Calculate the trajectory that will be followed. 
   * This should be called ahead of time so as not to waste time in auto.
  */
  public static void calcTrajectory(FileLog log) {
    try {
      // use starting angle to control trajectory
      // gyro must be zeroed prior to starting
      double startAngle = 0.0;

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

      // the trajectory to follow (all units in meters)
      // start at the origin facing the +X direction
      // Y is distance forward

      double firstBallY = 1.4; // actual is 1.7
      double firstBallX = 3.1; // actual is 3.1
      double distFirstToLastBall = 1.82; // distance between first and last ball
      double firstBallXOffset = 0.5; // aim for before first ball by this much
      double endX = firstBallX + distFirstToLastBall + firstBallXOffset;
      double endY = firstBallY;
      
      // drive from line to trench (assumes starting facing trench and shooting backwards)
      trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(startAngle)),
        List.of(
          new Translation2d(firstBallX-firstBallXOffset, firstBallY)
        ), 
        new Pose2d(endX, endY, new Rotation2d(startAngle)), config);


      // trajectory = TrajectoryGenerator.generateTrajectory(
      //   new Pose2d(0, 0, new Rotation2d(startAngle)),
      //   List.of(
      //     new Translation2d(-1.5,-.5)
      //   ), 
      //   new Pose2d(-2,-1, new Rotation2d(startAngle+180)), config);
    

      // dump trajectory for debugging
      for (State s : trajectory.getStates()) {
        var pose = s.poseMeters;
        var translation = pose.getTranslation();
        var rotation = pose.getRotation();

        log.writeLogEcho(true, "AutoTrench", "Trajectory", 
          "Time", s.timeSeconds, 
          "x", translation.getX(), 
          "y", translation.getY(), 
          "degrees", rotation.getDegrees(), 
          "velocityMetersPerSec", s.velocityMetersPerSecond);
      
      }
    } catch (Exception e) {
      log.writeLogEcho(true, "AutoTrench", "Trajectory", 
        "ERROR in calcTrajectory", e.toString(),"exception",e);
    }

    if (trajectory != null) {
      log.writeLogEcho(true, "AutoTrench", "Trajectory", "SUCCESS", true);
    };

  }

}
