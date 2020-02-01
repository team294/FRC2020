/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;

/**
 * Command to set the hopper percent output.
 */
public class HopperSetPercentOutput extends CommandBase {
  private Hopper hopper;
  private double percent;

  /**
   * @param percent percent output (0 to 1)
   * @param hopper hopper subsystem to use
   */
  public HopperSetPercentOutput(double percent, Hopper hopper) {
    this.hopper = hopper;
    this.percent = percent;
    addRequirements(hopper);
  }

  /**
   * Set the hopper percent output to default percent output from constants.
   * @param hopper hopper subsystem to use
   */
  public HopperSetPercentOutput(Hopper hopper) {
    this.hopper = hopper;
    this.percent = Constants.HopperConstants.hopperDefaultPercentOutput;
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hopper.hopperSetPercentOutput(percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) hopper.hopperSetPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
