/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoOponentTrenchPickup extends SequentialCommandGroup {
  /**
   * Creates a new AutoOponentTrenchPickup.
   */
  public AutoOponentTrenchPickup(Trajectory trajectory, DriveTrain driveTrain, LimeLight limeLight, FileLog log, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(

      new ParallelDeadlineGroup(
        new DriveStraight(3.2512, 0.5, 1.0, true, driveTrain, log),
        new IntakePistonSetPosition(true, intake),
        new IntakeSetPercentOutput(intake)
      ),
      
      new DriveFollowTrajectory(trajectory, true, true, driveTrain, log)
          .andThen(() -> driveTrain.tankDrive(0.0, 0.0, false)),

      new ParallelDeadlineGroup(new DriveTurnGyro(120, 0.5, 1.0, false, true, 2, driveTrain, limeLight, log), new ShooterSetPID(3000, shooter, led)),
      new DriveTurnGyro(120, 0.5, 1.0, true, true, 0.8, driveTrain, limeLight, log),

      new ParallelDeadlineGroup(
        new WaitForPowerCells(5, shooter),
        new ShooterFeederHopperSequence(3000, shooter, feeder, hopper, intake, led)
      ),
      
      new ParallelDeadlineGroup(
        new Wait(0.5),
        new ShooterFeederHopperIntakeStop(shooter, feeder, hopper, intake)
      )
      
    );
  }
}
