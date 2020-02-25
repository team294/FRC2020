/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.TargetType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class VisionAssistSequence extends SequentialCommandGroup {
  /**
   * Creates a new VisionAssistSequence.
   */

  public VisionAssistSequence(DriveTrain driveTrain, LimeLight limeLight, FileLog log, Shooter shooter, Feeder feeder, LED led, Hopper hopper, Intake intake) {
    super();
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(
      new DriveTurnGyro(TargetType.kVision, 0, 2.61, 3.8, 4, driveTrain, limeLight, log), // turn towards the general target
      new DriveStraight(limeLight.getSweetSpot(), TargetType.kVision, 0, 10, 10, true, driveTrain, limeLight, log),
      new DriveTurnGyro(TargetType.kVision, 0, 2.61, 3.8, 1, driveTrain, limeLight, log),

      new ParallelDeadlineGroup(
        new WaitForPowerCells(5, shooter),
        new ShooterFeederHopperSequence(3000, shooter, feeder, hopper, intake, led) // shoot until we shot 5 balls
      ),
      
      new ParallelDeadlineGroup(
        new Wait(0.1),
        new ShooterFeederHopperIntakeStop(shooter, feeder, hopper, intake, led) // stop all motors
      )
      
    );
    
  }
}
