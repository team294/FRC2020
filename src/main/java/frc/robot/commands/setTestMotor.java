/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Test;

public class setTestMotor extends CommandBase {
  /**
   * Creates a new setTestMotor.
   */
  private Test test;
  private double percent;

  public setTestMotor(Test test, double percent) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.test = test;
    this.percent = percent;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    test.setMotor(percent);
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
