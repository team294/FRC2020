/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

public class ClimbLift extends CommandBase {
  
  private Climb climb;
  private FileLog log;
  private double leftCurrHeight;
  private double rightCurrHeight;
  private double leftTarget;
  private double rightTarget;

  /**
   * Creates a new ClimbLift.
   */
  public ClimbLift(double target, Climb climb, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climb = climb;
    this.log = log;
    this.leftTarget = target;
    this.rightTarget = target;

    addRequirements(climb);
  }

  public ClimbLift(double leftTarget, double rightTarget,Climb climb, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climb = climb;
    this.log = log;
    this.leftTarget = leftTarget;
    this.rightTarget = rightTarget;

    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftCurrHeight = climb.getLeftEncoderInches();
    rightCurrHeight = climb.getRightEncoderInches();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.climbMotorLeftSetPosition(leftTarget);
    climb.climbMotorRightSetPosition(rightTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(leftCurrHeight - leftTarget) < 1 && Math.abs(rightCurrHeight - rightTarget) < 1) {
      return true;
    }
    
    return false;
  }
}
