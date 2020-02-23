/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

public class ShooterHoodPistonSequence extends SequentialCommandGroup {
  /**
   * Open or close shooter hood sequence.
   * @param close true = close hood, false = open hood
   * @param shooter shooter subsystem
   */
  public ShooterHoodPistonSequence(boolean close, Shooter shooter) { // TODO fix to have the correct naming of boolean
    addCommands(
      new ShooterSetLockPiston(true, shooter),
      // If closing, wait 0.5 seconds before setting hood piston. If opening, do not wait before setting hood piston.
      new ConditionalCommand(new Wait(0.5), new Wait(0), () -> close),
      new ShooterSetHoodPiston(close, shooter),
      new Wait(1),
      // If closing, do not lock hood. If opening, lock hood (extend lock piston).
      new ConditionalCommand(new Wait(0), new ShooterSetLockPiston(false, shooter), () -> close)
    );
  }
}
