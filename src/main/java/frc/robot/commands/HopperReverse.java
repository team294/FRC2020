/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.Hopper;

/**
 * Command to reverse the hopper every 1 second.
 */
public class HopperReverse extends CommandBase {
  private Hopper hopper;
  private Timer timerReverse, timerForward;

  /**
   * @param hopper hopper subsystem to use
   */
  public HopperReverse(Hopper hopper) {
    this.hopper = hopper;
    timerReverse = new Timer();
    timerForward = new Timer();
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timerReverse.reset();
    timerForward.reset();
    timerForward.start(); // start forward timer
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if hopper has been running forward for 1 second, run reverse and start reverse timer
    if (timerForward.hasPeriodPassed(1)) {
      timerReverse.start();
      timerForward.stop();
      hopper.hopperSetPercentOutput(-1 * HopperConstants.hopperDefaultPercentOutput);
      
      // if hopper has been running reverse for 1 second, run forward and reset both timers and start forward timer
    } else if (timerReverse.hasPeriodPassed(1)) {
      timerReverse.stop();
      timerReverse.reset();
      timerForward.reset();
      timerForward.start();
      hopper.hopperSetPercentOutput(HopperConstants.hopperDefaultPercentOutput);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timerReverse.stop();
    timerForward.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
