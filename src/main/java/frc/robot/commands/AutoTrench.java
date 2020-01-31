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

public class AutoTrench extends SequentialCommandGroup {

  // holds the trajectory we are going to follow
  private static Trajectory trajectory;

  // holds the kinematics for this driveTrain
  private static DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH);

  public AutoTrench(DriveTrain driveTrain) {

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
  }

  public static void calcTrajectory() {

    // use starting angle to control trajectory
    // double startAngle = driveTrain.getGyroRotation();
    double startAngle = 0;
    // System.out.println(startAngle);

    // Create a voltage constraint to ensure we don't accelerate too fast
    DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA), driveKinematics,
        DriveConstants.MAX_VOLTAGE);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(driveKinematics)
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    // Start at the origin facing the +X direction
    // Y is distance forward
    trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(startAngle)),
        List.of(new Translation2d(1, 0)), new Pose2d(3, 0, new Rotation2d(startAngle)), config);

    // dump trajectory for debugging
    for (State s : trajectory.getStates()) {
      Pose2d pose = s.poseMeters;
      System.out.printf("%f %s %f %n", s.timeSeconds, pose, s.velocityMetersPerSecond);
    }
  }

}
