/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

//NOTE:NEED WAY TO GET PISTON POSITION AND GET PERCENT OUTPUT FOR INTAKE

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class TestIntakeStart extends CommandBase {
  /**
   * Creates a new FeederHopperIntakeTest.
   */

  private Intake intake;

  public TestIntakeStart(Intake intake) {

    this.intake = intake;

    // var timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeSetPercentOutput(0.6);
    intake.intakeSetPiston(true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    intake.intakeGetPercentOutput();
    if (intake.intakeGetPercentOutput() >=  0.5){
      return true;
  }
  return false;
}
}