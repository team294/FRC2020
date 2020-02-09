/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ColorWheelConstants;
import frc.robot.subsystems.ColorWheel;
import frc.robot.utilities.ColorSensor;

/**
 * Command to rotate color wheel an indicated number of half-rotations.
 */
public class ColorWheelRotate extends CommandBase {
  private int halves;
  private ColorWheel colorWheel;
  private ColorSensor colorSensor;
  private String initialColor, currentColor;
  private boolean changedColor = false;
  private Timer timer;
  private int colorCount = 0; // Counts the number of times initial color is seen by colorSensor.
                              // 1 = half rotation, 2 = full rotation.

  /**
   * @param halves half rotations to rotate color wheel
   * @param colorWheel color wheel subsystem to use
   * @param colorSensor color sensor utility to use
   */
  public ColorWheelRotate(int halves, ColorWheel colorWheel, ColorSensor colorSensor) {
    this.halves = halves;
    this.colorWheel = colorWheel;
    this.colorSensor = colorSensor;
    timer = new Timer();
    addRequirements(colorWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    initialColor = colorSensor.getColor();
    colorWheel.colorWheelSetVoltage(ColorWheelConstants.colorWheelRotationVoltage);
    colorCount = 0;
    changedColor = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentColor = colorSensor.getColor();

    if (changedColor && timer.get() == 0 && currentColor.equals(initialColor)) timer.start();
    else if (changedColor && timer.get() != 0 && !currentColor.equals(initialColor)) {
      timer.stop();
      timer.reset();
    }

    if (!initialColor.equals(currentColor)) changedColor = true;
    else if (changedColor && initialColor.equals(currentColor) && timer.hasPeriodPassed(ColorWheelConstants.timerCheck)) {
      colorCount++;
      changedColor = false; // must change back to false so colorCount does not increase multiple times in one slice.
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    colorWheel.colorWheelSetVoltage(0);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (colorCount >= halves) return true;
    else return false;
  }
}
