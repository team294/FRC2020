/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/**
 * Command to set the piston position of both climb arms.
 */
public class ClimbPistonsSetPosition extends CommandBase {
  private boolean extend;
  private Climb climb;
  
  /**
   * @param extend true = extend pistons, false = retract pistons
   * @param climb climb subsystem to use
   */
  public ClimbPistonsSetPosition(boolean extend, Climb climb) {
    this.extend = extend;
    this.climb = climb;
    addRequirements(climb);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.climbPistonsSetPosition(extend);
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
