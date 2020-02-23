/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakePistonSetPosition extends CommandBase {
  private Intake intake;
  private boolean deploy;
  
  /**
   * Set the intake piston position.
   * NOTE: this command immediately ends.
   * @param deploy true = deploy, false = retract
   * @param intake intake subsystem to use
   */
  public IntakePistonSetPosition(boolean deploy, Intake intake) {
    this.intake = intake;
    this.deploy = deploy;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intakeSetPiston(deploy);
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
