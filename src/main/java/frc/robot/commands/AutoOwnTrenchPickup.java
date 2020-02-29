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
import frc.robot.Constants.TargetType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoOwnTrenchPickup extends SequentialCommandGroup {

// start with front two wheels on auto line drive frame 14 in from the right wall driver perspective

  public AutoOwnTrenchPickup(double waitTime, DriveTrain driveTrain, LimeLight limeLight, FileLog log, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {
    
    addCommands(
      
      new DriveZeroGyro(driveTrain, log),

      new Wait(waitTime),

      deadline(
        
        new DriveStraight(-1.5494, TargetType.kRelative, 0.0, 2.61, 3.8, true, driveTrain, limeLight, log), // drive to edge of trench
        new ShooterSetPID(true, false, shooter, limeLight, led), // start shooter
        new IntakePistonSetPosition(true, intake) // deploy intake piston
      ),
      
      new DriveTurnGyro(TargetType.kVision, 0, 450.0, 200, 0.8, driveTrain, limeLight, log).withTimeout(2), // turn towards target w/ vision
      
     deadline(
       new WaitForPowerCells(3, shooter).withTimeout(4), // wait for 3 power cells to be shot
       new ShootSequence(true, shooter, feeder, hopper, intake, limeLight, led) // start shooter
      ),
        
      new ShootSequenceStop(shooter, feeder, hopper, intake, led).withTimeout(0.1), // stop all motors

      new DriveTurnGyro(TargetType.kAbsolute, 180, 400.0, 200, 1, driveTrain, limeLight, log).withTimeout(1.5), // turn towards trench

      deadline( // drive down trench with intake
        new DriveStraight(3.2, TargetType.kRelative, 0.0, 2.088, 3.8, true, driveTrain, limeLight, log),
        new IntakeSequence(intake)
      ),
      
      new DriveTurnGyro(TargetType.kAbsolute, 25, 400.0, 200, 4, driveTrain, limeLight, log),

      new ParallelDeadlineGroup(
        new DriveTurnGyro(TargetType.kVision, 0, 450.0, 200, 0.8, driveTrain, limeLight, log).withTimeout(2), // turn towards target w/ vision
        
        new ShooterSetPID(true, false, shooter, limeLight, led) // start shooter
      ),

      deadline(
        new WaitForPowerCells(3, shooter).withTimeout(4), // wait for 3 power cells to be shot
        new ShootSequence(true, shooter, feeder, hopper, intake, limeLight, led) // start shooter
      ),
      
      new ShootSequenceStop(shooter, feeder, hopper, intake, led).withTimeout(0.1), // stop all motors


      new ShootSequenceStop(shooter, feeder, hopper, intake, led).withTimeout(0.1) // stop all motors

    );
  }
}
