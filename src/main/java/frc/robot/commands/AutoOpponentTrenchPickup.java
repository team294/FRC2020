/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.TargetType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoOpponentTrenchPickup extends SequentialCommandGroup {


  // start robot infront of opponents trench with the intake facing the trench

  public AutoOpponentTrenchPickup(Trajectory trajectory, DriveTrain driveTrain, LimeLight limeLight, FileLog log, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(

      new DriveZeroGyro(180, driveTrain, log),

      new ParallelDeadlineGroup( // ends when we reach the two balls in the trench
        new DriveStraight(2.6, TargetType.kRelative, 0.0, 2.61, 3.8, true, driveTrain, limeLight, log), // drive forward into trench
        new IntakeSequence(intake)
      ),

      new ParallelDeadlineGroup( // ends when we reach the two balls in the trench
        new DriveStraight(-0.5, TargetType.kRelative, 0.0, 2.61, 3.8, true, driveTrain, limeLight, log), // drive forward into trench
        new IntakeSequence(intake)
      ),
      
      new DriveTurnGyro(TargetType.kRelative, -35, 300, 200, 2, driveTrain, limeLight, log),

      new ParallelDeadlineGroup( // ends when we reach the two balls in the trench
        new DriveStraight(0.75, TargetType.kRelative, 0.0, 2.61, 3.8, true, driveTrain, limeLight, log), // drive forward into trench
        new IntakeSequence(intake)
      ),

      new ParallelDeadlineGroup( // ends when we reach the two balls in the trench
        new DriveStraight(-2, TargetType.kRelative, 0.0, 2.61, 3.8, true, driveTrain, limeLight, log), // drive forward into trench
        new IntakeSequence(intake)
      ),

     // new DriveFollowTrajectory(CoordType.kRelative, trajectory, driveTrain, log) // run a path to get out of the trench and do a curve to get to shooting position 
       //   .andThen(() -> driveTrain.tankDrive(0.0, 0.0, false)),

      new ParallelDeadlineGroup(
          new DriveTurnGyro(TargetType.kAbsolute, -45, 400, 200, 3, driveTrain, limeLight, log), // turn towards the general target
          new ShooterSetPID(3000, shooter, led) // start shooter while shooting
        ), 

      new ParallelRaceGroup(
          new DriveTurnGyro(TargetType.kVision, 0, 400, 200, 0.8, driveTrain, limeLight, log), // turn towards target w/ vision
          new Wait(2)
        ),

      new ShooterHoodPistonSequence(true, false, shooter),
        
      new ParallelDeadlineGroup(
        new WaitForPowerCells(5, shooter),
        new ShootSequence(3000, shooter, feeder, hopper, intake, led) // shoot until we shot 5 balls
      ),
      
      new ParallelDeadlineGroup(
        new Wait(0.1),
        new ShootSequenceStop(shooter, feeder, hopper, intake, led) // stop all motors
      )
      
    );
  }
}
