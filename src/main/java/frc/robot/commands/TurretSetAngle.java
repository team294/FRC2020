/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

/**
 * Command to move turret to specific angle.
 */
public class TurretSetAngle extends CommandBase {
  private Turret turret;
  private double targetAngle;
  
  /**
   * @param angle target angle
   * @param turret turret subsystem to use
   */
  public TurretSetAngle(double angle, Turret turret) {
    this.turret = turret;
    addRequirements(turret);
    this.targetAngle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.turretSetAngle(targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.turretSetPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(turret.turretGetAngleError()) < 2) return true;
    else return false;
  }
}