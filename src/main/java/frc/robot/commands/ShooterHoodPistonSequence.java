/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

/**
 * Command sequence to open or close the shooter hood (with pistons).
 */
public class ShooterHoodPistonSequence extends SequentialCommandGroup {
  /**
   * @param open true = open the hood, false = close the hood
   */
  public ShooterHoodPistonSequence(boolean open, Shooter shooter) {
    addCommands(
      new ShooterSetLockPiston(true, shooter),
      new Wait(1),
      new ShooterSetHoodPiston(open, shooter)
    );
  }
}
