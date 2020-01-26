/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command to do nothing and end after a specified duration (useful for command groups).
 */
public class Wait extends CommandBase {
  private Timer timer;
  private double duration;

  /**
   * @param seconds timer duration, in seconds
   */
  public Wait(double seconds) {
    this.timer = new Timer();
    this.duration = seconds;
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.hasPeriodPassed(duration);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
