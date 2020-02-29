/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeSetPercentOutput extends CommandBase {
  private Intake intake;
  private double percent;
  private boolean end;

  /**
   * Set intake percent output using parameter percent.
   * This command never ends.
   * @param percent percent output (0 to 1)
   * @param intake intake subsystem to use
   */
  public IntakeSetPercentOutput(double percent, boolean end, Intake intake) {
    this.intake = intake;
    this.percent = percent;
    this.end = end;
    addRequirements(intake);
  }

  /**
   * Set intake percent output using default percent from constants.
   * This command never ends.
   * @param intake intake subsystem to use
   */
  public IntakeSetPercentOutput(boolean end, Intake intake) {
    this.intake = intake;
    this.percent = IntakeConstants.intakeDefaultPercentOutput;
    this.end = end;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intakeSetPercentOutput(percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) intake.intakeSetPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (end) return true;
    else return false;
  }
}
