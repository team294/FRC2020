package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;



/**
 * Command to follow a trajectory using an embedded Ramsete Command.
 */
public class DriveFollowTrajectoryOLD extends SequentialCommandGroup {

  // Stateless static variables common to all trajectory followers
  private static DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH);

  // Member variables for this object
  Pose2d poseRobot;       // Current pose of robot
  RamseteController ramseteController;

  /**
   * Follow a trajectory using an embedded Ramsete Command
   * @param driveTrain  The driveTrain to use to get the pose, wheel speeds and set the tankDriveVolts
   */  
  public DriveFollowTrajectoryOLD(Trajectory trajectory, DriveTrain driveTrain, FileLog log) {
    boolean bUseFeedback = true;
    boolean bUseRamsete = true;


    if (bUseRamsete) {
      ramseteController = new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta);
    } else {
      ramseteController = new RamseteController() {
        @Override
        public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters, double angularVelocityRefRadiansPerSecond) {
          return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
        }
      };
    }

    PIDController leftController = new PIDController(DriveConstants.kP, 0, 0);
    PIDController rightController = new PIDController(DriveConstants.kP, 0, 0);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Auto Paths");
    NetworkTableEntry velocityLeftSet = table.getEntry("Auto Vel Left Set");
    NetworkTableEntry velocityRightSet = table.getEntry("Auto Vel Right Set");
    NetworkTableEntry velocityLeftAct = table.getEntry("Auto Vel Left Act");
    NetworkTableEntry velocityRightAct = table.getEntry("Auto Vel Right Act");

    RamseteCommand ramseteFollower = new RamseteCommand(
      trajectory,
      // driveTrain::getPose,
      () -> {
        poseRobot = driveTrain.getPose();
        return poseRobot;
      },
      ramseteController,
      new SimpleMotorFeedforward(DriveConstants.kS,DriveConstants.kV,DriveConstants.kA),
      driveKinematics,
      driveTrain::getWheelSpeeds,
      leftController,
      rightController,
      //driveTrain::tankDriveVolts,
      (leftVolts, rightVolts) -> {
        driveTrain.tankDriveVolts(leftVolts, rightVolts);
        velocityLeftSet.setNumber(leftController.getSetpoint());
        velocityRightSet.setNumber(rightController.getSetpoint());
        velocityLeftAct.setNumber(driveTrain.getWheelSpeeds().leftMetersPerSecond);
        velocityRightAct.setNumber(driveTrain.getWheelSpeeds().rightMetersPerSecond);
        System.out.println( poseRobot.getTranslation().getX() );

        log.writeLog(true, "TankDriveVolts", "Update", 
          // "Time", autoTimer.get(), 
          "L Meters", Units.inchesToMeters(driveTrain.getLeftEncoderInches()),
          "R Meters", Units.inchesToMeters(driveTrain.getRightEncoderInches()), 
          "L Velocity", Units.inchesToMeters(driveTrain.getLeftEncoderVelocity()), 
          "R Velocity", Units.inchesToMeters(-driveTrain.getRightEncoderVelocity()), 
          "L Volts", leftVolts, 
          "R Volts", rightVolts, 
          "Robot Angle", poseRobot.getRotation().getDegrees());
      },
      driveTrain
    );
    
    // Run the ramsete command and then stop
    addCommands(
      ramseteFollower.andThen(() -> driveTrain.tankDrive(0.0, 0.0, false))
    );
  }



}
