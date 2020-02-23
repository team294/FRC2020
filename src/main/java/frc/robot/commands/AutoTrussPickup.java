/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoTrussPickup extends SequentialCommandGroup {
  /**
   * Creates a new AutoTrussPickup.
   */
  public AutoTrussPickup(DriveTrain driveTrain, LimeLight limeLight, FileLog log, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {
    
    // start with edge of bumpers to the left edge of the center line, intake facing towards truss, line up straight
    
    addCommands(

      new DriveZeroGyro(180, driveTrain),

      new ParallelDeadlineGroup(
        new DriveStraight(2.08, 0.5, 1.0, true, driveTrain, log), // drive to 2 of balls on truss
        new IntakePistonSetPosition(true, intake), // deploy intake piston
        new IntakeSetPercentOutput(intake) // spin intake
      ),

      new ParallelDeadlineGroup(
        new DriveStraight(-0.5, 0.5, 1, true, driveTrain, log), // back up a short ammount 
        new IntakeSequence(intake) // keep intake spinning
      ),
      //new DriveStraight(-0.5, 0.5, 1, true, driveTrain, log),

      new ParallelDeadlineGroup(
        new DriveTurnGyro(180, 0.6, 1.0, false, true, true, 3, driveTrain, limeLight, log), // turn towards general target
        new ShooterSetPID(3000, shooter, led) // start shooter motors
      ),

      new ParallelRaceGroup(
          new DriveTurnGyro(0, 0.5, 1.0, true, true, false, 0.8, driveTrain, limeLight, log), // turn towards target w/ vision
          new Wait(2)
        ),
        
      new ParallelDeadlineGroup(
        new ParallelRaceGroup(
          new WaitForPowerCells(5, shooter), 
          new Wait(7)
        ),  // wait for 5 balls to be shot or 7 seconds
        new ShooterFeederHopperSequence(3000, shooter, feeder, hopper, intake, led) // shoot
      ),
      
      new ParallelDeadlineGroup(
        new Wait(0.1),
        new ShooterFeederHopperIntakeStop(shooter, feeder, hopper, intake, led) // stop all motors
      )
      
    );
  }
}
