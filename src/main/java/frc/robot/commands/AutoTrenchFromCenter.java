package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;


/**
 * Auto routine starting at the initiation line directly in front of the target and getting balls from the trench
 */
public class AutoTrenchFromCenter extends SequentialCommandGroup {

  /**
   * AutoTrench constructor for command group
   * @param driveTrain  The driveTrain to use to get the pose, wheel speeds and set the tankDriveVolts
   */  
  public AutoTrenchFromCenter(DriveTrain driveTrain, FileLog log, Trajectory trajectory, DifferentialDriveKinematics driveKinematics) {

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



}
