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

public class ClimbSetVelocity extends CommandBase {
  private Climb climb;
  private double velocity, position, timeRemaining;

  /**
   * Set both climb arm velocities.
   * This command ends when the right climb arm gets within the tolerance of the target position.
   * @param velocity velocity (inches/second)
   * @param position target position (inches)
   * @param climb climb subsystem
   */
  public ClimbSetVelocity(double velocity, double position, Climb climb) {
    this.climb = climb;
    this.velocity = velocity;
    this.position = position;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeRemaining = DriverStation.getInstance().getMatchTime();
    // if it is the last 30 seconds of the match and the piston is extended, set right motor velocity
    if (/*timeRemaining <= 30 && */climb.climbPistonsGetPosition()) {
      climb.climbMotorsSetVelocity(velocity);
    }
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
    // if it is the last 30 seconds of the match and the piston is extended, check for being at target position
    if (/*timeRemaining <= 30 && */climb.climbPistonsGetPosition()) {
      if (Math.abs(climb.getRightEncoderInches() - position) <= ClimbConstants.positionTolerance) climb.climbMotorRightSetPercentOutput(0);
      if (Math.abs(climb.getLeftEncoderInches() - position) <= ClimbConstants.positionTolerance) climb.climbMotorLeftSetPercentOutput(0);
      
      if ((Math.abs(climb.getRightEncoderInches() - position) <= ClimbConstants.positionTolerance)
        && (Math.abs(climb.getLeftEncoderInches() - position) <= ClimbConstants.positionTolerance)) return true;
      else return false;
    } else return true;
  }
}
