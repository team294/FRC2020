package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;


/**
 * Auto routine starting at the initiation line directly in front of the target and getting balls from the trench
 */
public class AutoTrenchFromCenter extends SequentialCommandGroup {

  /**
   * AutoTrench constructor for command group
   * @param driveTrain  The driveTrain to use to get the pose, wheel speeds and set the tankDriveVolts
   */  
  public AutoTrenchFromCenter(Trajectory trajectory, DifferentialDriveKinematics driveKinematics, DriveTrain driveTrain, Shooter shooter, Feeder feeder, Hopper hopper, 
    Intake intake, LimeLight limeLight, FileLog log) {

    addCommands(
      /*
      new ParallelDeadlineGroup(
        new Wait(2),
        new ShooterFeederHopperSequenceNoPiston(shooter, feeder, hopper, intake)
      ),
      new ParallelDeadlineGroup(
        new Wait(0.5),
        new ShooterFeederHopperIntakeStop(shooter, feeder, hopper, intake)
      ),
      //new DriveTurnGyro(180, 0.8, 1.0, false, true, driveTrain, limeLight, log),
      new ParallelDeadlineGroup(
      */

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
        ).andThen(() -> driveTrain.tankDrive(0.0, 0.0, false)


        /*  ),
        new IntakeSetPercentOutput(intake)
        */


      )
      /*
      ,
      new ShooterFeederHopperIntakeStop(shooter, feeder, hopper, intake),
      new DriveTurnGyro(175, 0.8, 1.0, false, true, driveTrain, limeLight, log),
      new DriveStraight(1.5, 0.5, 0.5, true, driveTrain, log),
      new DriveTurnGyro(0, 0.4, 1.0, true, true, driveTrain, limeLight, log),
      new ShooterFeederHopperSequenceNoPiston(shooter, feeder, hopper, intake)
      */
    );
  }



}
