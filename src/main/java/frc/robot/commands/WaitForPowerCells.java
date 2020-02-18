/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/**
 * Command to wait until a specified number of power cells are shot.
 */
public class WaitForPowerCells extends CommandBase {
  private int cells;
  private Shooter shooter;

  /**
   * @param cells number of power cells to wait for to finish
   * @param shooter shooter subsystem to use
   */
  public WaitForPowerCells(int cells, Shooter shooter) {
    this.cells = cells;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (cells >= shooter.getPowerCellsShot()) return true;
    else return false;
  }
}
