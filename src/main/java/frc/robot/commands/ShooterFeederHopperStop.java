/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

/**
 * Command group to stop the shooter, feeder, and hopper.
 */
public class ShooterFeederHopperStop extends SequentialCommandGroup {
  /**
   * @param shooter shooter subsystem to use
   * @param feeder feeder subsystem to use
   * @param hopper hopper subsystem to use
   */
  public ShooterFeederHopperStop(Shooter shooter, Feeder feeder, Hopper hopper) {
    addCommands(
      new FeederSetPiston(false, feeder),
      new ParallelCommandGroup(
        new ShooterSetVoltage(0, shooter),
        new FeederSetVoltage(0, feeder),
        new HopperSetPercentOutput(0, hopper)
      )
    );
  }
}
