/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class ClimbRightSetPercentOutput extends CommandBase {
  private Climb climb;
  private double percent, timeRemaining;

  /**
   * Set percent output of left climb motor.
   * This command never ends.
   * @param percent percent output (0 to 1)
   * @param climb climb subsystem
   */
  public ClimbRightSetPercentOutput(double percent, Climb climb) {
    this.climb = climb;
    this.percent = percent;
    timeRemaining = DriverStation.getInstance().getMatchTime();
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if it is the last 30 seconds of the match and the piston is extended, set left and right motor perecnt output
    if (/*timeRemaining <= 30 && */climb.climbPistonsGetPosition())
      climb.climbMotorRightSetPercentOutput(percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.climbMotorRightSetPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if it is the last 30 seconds of the match and the piston is extended, do not finish
    /* if (timeRemaining <= 30 && climb.climbPistonsGetPosition()) return false; */
    if (climb.getRightEncoderInches() >= ClimbConstants.maxHeight && percent > 0) return true;
    else return false;
  }
}
