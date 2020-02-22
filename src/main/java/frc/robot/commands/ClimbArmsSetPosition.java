/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.*;

/**
 * Command to set the position of both climb arms.
 */
public class ClimbArmsSetPosition extends CommandBase {
  private Climb climb;
  private double leftPosition, rightPosition;
  private double leftTarget, rightTarget;

  /**
   * @param target target position, in inches
   * @param climb climb subsystem to use
   */
  public ClimbArmsSetPosition(double target, Climb climb/*, FileLog log*/) {
    this.climb = climb;
    this.leftTarget = target;
    this.rightTarget = target;

    addRequirements(climb);
  }

  /**
   * @param leftTarget left arm target position, in inches
   * @param rightTarget right arm target position, in inches
   * @param climb climb subsystem to use
   */
  public ClimbArmsSetPosition(double leftTarget, double rightTarget, Climb climb) {
    this.climb = climb;
    this.leftTarget = leftTarget;
    this.rightTarget = rightTarget;

    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftPosition = climb.getLeftEncoderInches();
    rightPosition = climb.getRightEncoderInches();

    climb.climbMotorLeftSetPosition(leftTarget);
    climb.climbMotorRightSetPosition(rightTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftPosition = climb.getLeftEncoderInches();
    rightPosition = climb.getRightEncoderInches();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(leftPosition - leftTarget) < ClimbConstants.positionTolerance 
      && Math.abs(rightPosition - rightTarget) < ClimbConstants.positionTolerance) {
      return true;
    } else return false;
  }
}
