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
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoOponentTrenchPickup extends SequentialCommandGroup {


  // start robot infront of opponents trench with the intake facing the trench

  public AutoOponentTrenchPickup(Trajectory trajectory, DriveTrain driveTrain, LimeLight limeLight, FileLog log, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(
      
      deadline( // ends when we reach the two balls in the trench
        new DriveStraight(3.2512, 0.5, 1.0, true, driveTrain, log), // drive forward into trench
        new IntakePistonSetPosition(true, intake), // depoy intake piston
        new IntakeSetPercentOutput(intake) // start intake 
      ),
      
      new DriveFollowTrajectory(trajectory, driveTrain, log), // run a path to get out of the trench and do a curve to get to shooting position 

      deadline(
          new DriveTurnGyro(120, 0.5, 1.0, false, true, 2, driveTrain, limeLight, log), // turn towards the general target
          new ShooterSetPID(3000, shooter, led) // start shooter while shooting
      ),

      new DriveTurnGyro(0, 0.5, 1.0, true, true, 0.8, driveTrain, limeLight, log).withTimeout(2), // turn towards target w/ vision
          
        
     deadline(
        new WaitForPowerCells(5, shooter),
        new ShooterFeederHopperSequence(3000, shooter, feeder, hopper, intake, led)// shoot until we shot 5 balls
      ),
      
      new ShooterFeederHopperIntakeStop(shooter, feeder, hopper, intake, led).withTimeout(0.1) // stop all motors
      
    );
  }
}
