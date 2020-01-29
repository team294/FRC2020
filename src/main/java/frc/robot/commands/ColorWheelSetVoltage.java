/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheel;

/**
 * Command to set voltage of color wheel.
 */
public class ColorWheelSetVoltage extends CommandBase {
  private final ColorWheel colorWheel;
  private final double voltage;

  /**
   * Sets the shooter voltage.
   * @param voltage +12 (full forward) to -12 (full reverse)
   * @param colorWheel color wheel subsystem to use
   */
  public ColorWheelSetVoltage(double voltage, ColorWheel colorWheel) {
    this.colorWheel = colorWheel;
    this.voltage = voltage;
    addRequirements(colorWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorWheel.colorWheelSetVoltage(voltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    colorWheel.colorWheelSetVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
