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
 * Command sequence to get the shooter and feeder up to speed, and then run the hopper.
 */
public class ShooterFeederHopperSequence extends SequentialCommandGroup {
  /**
   * @param shooter shooter subsystem to use
   * @param feeder feeder subsystem to use
   * @param hopper hopper subsystem to use
   */
  public ShooterFeederHopperSequence(Shooter shooter, Feeder feeder, Hopper hopper) {
    addCommands( 
      new ParallelCommandGroup(new ShooterSetPID(shooter), new FeederSetPID(feeder),
      new HopperSetPercentOutput(hopper))
    );
  }
}
