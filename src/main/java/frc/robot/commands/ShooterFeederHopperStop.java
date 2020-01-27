/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShooterFeederHopperStop extends ParallelCommandGroup {
  /**
   * Creates a new ShooterFeederHopperStop.
   */
  public ShooterFeederHopperStop(Shooter shooter, Feeder feeder, Hopper hopper) {
    addCommands(
      new ShooterSetVoltage(0, shooter),
      new FeederSetVoltage(0, feeder),
      new HopperSetPercentOutput(0, hopper)
    );
  }
}
