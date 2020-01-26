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
 * Command to set the shooter voltage.
 */
public class ShooterSetVoltage extends CommandBase {
  private final Shooter shooter;
  private final double voltage;

  /**
   * @param voltage +12 (full forward) to -12 (full reverse)
   * @param shooter shooter subsystem to use
   */
  public ShooterSetVoltage(double voltage, Shooter shooter) {
    this.shooter = shooter;
    this.voltage = voltage;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterVoltage(voltage);
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
    return true;
  }
}
