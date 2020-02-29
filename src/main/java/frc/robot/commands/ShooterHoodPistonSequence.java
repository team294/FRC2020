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
   * @param lock true = lock hood at the end, false = do not lock hood at the end
   * @param shooter shooter subsystem
   */
  public ShooterHoodPistonSequence(boolean close, boolean lock, Shooter shooter) {
    addCommands(
      new ConditionalCommand(
        // if hood is already not in position, do entire piston sequence
        sequence(
          new ShooterSetLockPiston(true, shooter),
          // If opening, wait 0.5 seconds before setting hood piston. If closing, do not wait before setting hood piston.
          new ConditionalCommand(new Wait(0.5), new Wait(0), () -> !close),
          new ShooterSetHoodPiston(close, shooter),
          new Wait(0.75),
          // If opening or parameter lock is false, do not lock hood. If closing and parameter lock is true, lock hood (extend lock piston).
          new ConditionalCommand(new ShooterSetLockPiston(true, shooter), new ShooterSetLockPiston(false, shooter), () -> !close || !lock)
        ), 
        // otherwise, lock or unlock the hood if needed
        new ConditionalCommand(
          new ShooterSetLockPiston(true, shooter), 
          new ShooterSetLockPiston(false, shooter), 
          () -> !close || !lock
        ),
        () -> shooter.getHoodPiston() != close
      )
    );
  }
}

      /*
      new ShooterSetLockPiston(true, shooter),
      // If opening, wait 0.5 seconds before setting hood piston. If closing, do not wait before setting hood piston.
      new ConditionalCommand(new Wait(0.5), new Wait(0), () -> !close),
      new ShooterSetHoodPiston(close, shooter),
      new Wait(0.75),
      // If opening or parameter lock is false, do not lock hood. If closing and parameter lock is true, lock hood (extend lock piston).
      new ConditionalCommand(new ShooterSetLockPiston(true, shooter), new ShooterSetLockPiston(false, shooter), () -> !close || !lock)
      */

      /*new ConditionalCommand(
        // If hood and lock are already in place, end immediately.
        new Wait(0),
        new ConditionalCommand(
          // If hood is already in place but lock is not, set lock position.
          new ShooterSetLockPiston(!lock, shooter), 
          // Otherwise, do normal hood movement sequence.
          sequence(
            // unlock hood
            new ShooterSetLockPiston(true, shooter),
            // If opening, delay before setting hood piston. If closing, do not wait before setting hood piston.
            new ConditionalCommand(new Wait(0.5), new Wait(0), () -> !close),
            // close or open hood
            new ShooterSetHoodPiston(close, shooter),
            // If opening or parameter lock is false, do not lock hood. If closing and parameter lock is true, delay and lock hood.
            new ConditionalCommand(
              new ShooterSetLockPiston(true, shooter), 
              parallel(new Wait(0.75), new ShooterSetLockPiston(false, shooter)), 
                () -> !close || !lock
            )
          ), () -> close == !shooter.getHoodPiston() && lock == !shooter.getLockPiston() // condition for hood being already in place but lock is not
        ), () -> close == !shooter.getHoodPiston() && lock == shooter.getLockPiston() // condition for hood and lock already being in place
      )*/
