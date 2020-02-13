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

    RamseteController disabledRamsete = new RamseteController() {
      @Override
      public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters, double angularVelocityRefRadiansPerSecond) {
        return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
      }
    };

    PIDController leftController = new PIDController(DriveConstants.kP, 0, 0);
    PIDController rightController = new PIDController(DriveConstants.kP, 0, 0);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Auto Paths");
    NetworkTableEntry velocityLeftSet = table.getEntry("Auto Vel Left Set");
    NetworkTableEntry velocityRightSet = table.getEntry("Auto Vel Right Set");
    NetworkTableEntry velocityLeftAct = table.getEntry("Auto Vel Left Act");
    NetworkTableEntry velocityRightAct = table.getEntry("Auto Vel Right Act");

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
          //disabledRamsete,
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
          },
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
