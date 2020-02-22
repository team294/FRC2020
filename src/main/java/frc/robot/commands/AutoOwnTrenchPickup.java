/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoOwnTrenchPickup extends SequentialCommandGroup {

// start with front two wheels on auto line drive frame 14 in from the right wall driver perspective

  public AutoOwnTrenchPickup(DriveTrain driveTrain, LimeLight limeLight, FileLog log, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {
    
    addCommands(
    
      deadline(
        new DriveStraight(-1.5494, 0.5, 1.0, true, driveTrain, log), // drive to edge of trench
        new ShooterSetPID(2800, shooter, led), // start shooter
        new IntakePistonSetPosition(true, intake) // deploy intake piston
      ),
      
      new DriveTurnGyro(0, 0.5, 1.0, true, true, 0.8, driveTrain, limeLight, log).withTimeout(2), // turn towards target w/ vision
        
     deadline(
        new WaitForPowerCells(3, shooter).withTimeout(4), // wait for 3 power cells to be shot
        new ShooterFeederHopperSequence(2800, shooter, feeder, hopper, intake, led) // start shooter
      ),

      new ShooterFeederHopperIntakeStop(shooter, feeder, hopper, intake, led).withTimeout(0.1), // stop all motors

      new DriveTurnGyro(-163, 0.8, 1, false, true, 1, driveTrain, limeLight, log).withTimeout(1.5), // turn towards trench
    
      deadline( // drive down trench with intake
        new DriveStraight(3.2, 0.4, 1.0, true, driveTrain, log),
        new IntakeSequence(intake)
      ),
      
      //new DriveStraight(-2, 0.5, 1.0, true, driveTrain, log),

      new DriveTurnGyro(165, 0.8, 1.0, false, true, 4, driveTrain, limeLight, log),

      deadline(
        new DriveTurnGyro(0, 0.5, 1.0, true, true, 0.8, driveTrain, limeLight, log).withTimeout(2), // turn towards target w/ vision
        new ShooterSetPID(3500, shooter, led) // start shooter
      ),

      deadline(
        new WaitForPowerCells(3, shooter).withTimeout(4), // wait for 3 power cells to be shot
        new ShooterFeederHopperSequence(3000, shooter, feeder, hopper, intake, led) // start shooter
      ),

      new ShooterFeederHopperIntakeStop(shooter, feeder, hopper, intake, led).withTimeout(0.1) // stop all motors

    );
  }
}
