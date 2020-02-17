package frc.robot.commands;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

/**
 * Auto routine starting at the initiation line right position and getting balls from the trench
 */
public class AutoTrenchFromRight extends SequentialCommandGroup {

  /**
   * AutoTrench constructor for command group
   * @param driveTrain  The driveTrain to use to get the pose, wheel speeds and set the tankDriveVolts
   */  
  public AutoTrenchFromRight(DriveTrain driveTrain, FileLog log, Trajectory trajectory) {

    addCommands(
      new DriveFollowTrajectory(trajectory, true, true, driveTrain, log)
        .andThen(() -> driveTrain.tankDrive(0.0, 0.0, false))
    );

  }
}
