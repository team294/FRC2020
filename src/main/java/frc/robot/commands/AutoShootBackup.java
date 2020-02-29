/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TargetType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShootBackup extends SequentialCommandGroup {
  /**
   * Creates a new AutoShootBackup.
   */
  public AutoShootBackup(double waitTime, DriveTrain driveTrain, LimeLight limeLight, FileLog log, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {

    // can start anywhere on auto line between left most pole from driver perspective and close to right edge of the field, needs to be semi lined up with target
    // one front wheel on line, one behind

    addCommands(

      new Wait(waitTime),

      deadline(
        new DriveTurnGyro(TargetType.kVision, 0, 450, 200, 0.8, driveTrain, limeLight, log).withTimeout(2), // turn towards target w/ vision
        new ShooterSetPID(true, false, shooter, limeLight, led), // start shooter
        new IntakePistonSetPosition(true, intake) // deploy intake piston
      ),

      new ShooterHoodPistonSequence(true, false, shooter),

       deadline(
        new WaitForPowerCells(3, shooter).withTimeout(7), // wait for 3 power cells to be shot
        new ShootSequence(true, shooter, feeder, hopper, intake, limeLight, led) // start shooter
      ),

      parallel(
        new ShootSequenceStop(shooter, feeder, hopper, intake, led).withTimeout(0.1), // stop all motors
      
        new DriveStraight(-1, TargetType.kRelative, 0.0, 2.61, 3.8, true, driveTrain, limeLight, log) // back up 1 meter to get off auto line
      )
    );
  }
}
