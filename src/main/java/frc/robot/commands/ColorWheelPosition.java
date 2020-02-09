/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ColorWheelConstants;
import frc.robot.subsystems.ColorWheel;
import frc.robot.utilities.ColorSensor;

/**
 * Command to spin control panel to an indicated color.
 */
public class ColorWheelPosition extends CommandBase {
  private String fmsColor;
  private ColorWheel colorWheel;
  private ColorSensor colorSensor;
  private String currentColor, previousColor;
  /*private String initialColor;
  private boolean changedColor = false;*/
  private Timer timer;

  /**
   * @param colorWheel color wheel subsystem to use
   * @param colorSensor color sensor utility to use
   */
  public ColorWheelPosition(ColorWheel colorWheel, ColorSensor colorSensor) {
    this.colorWheel = colorWheel;
    this.colorSensor = colorSensor;
    fmsColor = DriverStation.getInstance().getGameSpecificMessage().toString();
    timer = new Timer();
    addRequirements(colorWheel);
  }

  /**
   * @param fmsColor color to turn to
   * @param colorWheel color wheel subsystem to use
   * @param colorSensor color sensor utility to use
   */
  public ColorWheelPosition(String fmsColor, ColorWheel colorWheel, ColorSensor colorSensor) {
    this.fmsColor = fmsColor;
    this.colorWheel = colorWheel;
    this.colorSensor = colorSensor;
    timer = new Timer();
    addRequirements(colorWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    //initialColor = colorSensor.getColor();
    colorWheel.colorWheelSetVoltage(ColorWheelConstants.colorWheelPositionVoltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    previousColor = currentColor;
    currentColor = colorSensor.getColor();

    if (previousColor.equals(currentColor) && timer.get() == 0) timer.start();
    else if (!previousColor.equals(currentColor) && timer.get() != 0 && timer.get() > 0.008) {
      timer.stop();
      timer.reset();
    }

    /*if (!initialColor.equals(currentColor)) changedColor = true;
    
    if (changedColor && timer.get() == 0 && colorSensor.getControlPanelSensorColor().equals(fmsColor)) timer.start();
    else if (changedColor && timer.get() != 0 && !colorSensor.getControlPanelSensorColor().equals(fmsColor)) {
      timer.stop();
      timer.reset();
    }*/
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
    if (timer.get() > ColorWheelConstants.timerCheck && colorSensor.getControlPanelSensorColor().equals(fmsColor) 
      || fmsColor.length() == 0) return true;
    else return false;

    /*if ((timer.hasPeriodPassed(Constants.ColorWheelConstants.timerCheck) && colorSensor.getControlPanelSensorColor().equals(fmsColor)) 
      || fmsColor.length() == 0) return true;
    else return false;*/
  }
}
