/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TestSequence extends SequentialCommandGroup {
  /**
   * Creates a new TestA.
   */

  public TestSequence(Shooter shooter, Feeder feeder, Hopper hopper, Intake intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(
     new TestFeederStart(feeder),
     new TestFeederEnd(feeder),
     new TestHopperStart(hopper),
     new TestHopperEnd(hopper),
      //new TestIntakeStart(intake),
      //new TestIntakeEnd(intake),    //parts of code for intake missing for testing
     new TestShooterStart(shooter),
     new TestShooterFinish(shooter)
    );
  }
}
