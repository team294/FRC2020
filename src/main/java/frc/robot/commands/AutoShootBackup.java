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
public class AutoShootBackup extends SequentialCommandGroup {
  /**
   * Creates a new AutoShootBackup.
   */
  public AutoShootBackup(DriveTrain driveTrain, LimeLight limeLight, FileLog log, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {

    // can start anywhere on auto line between left most pole from driver perspective and close to right edge of the field, needs to be semi lined up with target

    addCommands(
      new ParallelDeadlineGroup(
        new ParallelRaceGroup(
          new DriveTurnGyro(0, 0.5, 1.0, true, true, 0.8, driveTrain, limeLight, log), // turn towards target w/ vision
          new Wait(2)
        ),
        
        new ShooterSetPID(2800, shooter, led), // start shooter
        new IntakePistonSetPosition(true, intake) // deploy intake piston
      ),

      new ParallelDeadlineGroup(
        new ParallelRaceGroup(
          new WaitForPowerCells(3, shooter), // wait for 3 power cells to be shot
          new Wait(10)
        ), 
        new ShooterFeederHopperSequence(2800, shooter, feeder, hopper, intake, led) // start shooter
      ),
      new ParallelDeadlineGroup(
        new Wait(0.1),
        new ShooterFeederHopperIntakeStop(shooter, feeder, hopper, intake, led) // stop all motors
      ),
      
      new DriveStraight(-1, 0.5, 1.0, true, driveTrain, log) // back up 1 meter to get off auto line

    );
  }
}