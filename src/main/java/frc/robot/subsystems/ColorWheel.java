/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ColorSensor;
import frc.robot.utilities.FileLog;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import static frc.robot.Constants.ColorWheelConstants.*;

public class ColorWheel extends SubsystemBase {
  private final WPI_TalonSRX colorWheelMotor = new WPI_TalonSRX(canColorWheelMotor);
  private ColorSensor colorSensor;
  private FileLog log;

  /**
   * @param colorSensor color sensor utility to use
   */
  public ColorWheel(ColorSensor colorSensor, FileLog log) {
    this.colorSensor = colorSensor;
    this.log = log; // save reference to the fileLog
  }

  /**
   * @param voltage voltage
   */
  public void colorWheelSetVoltage(double voltage) {
    colorWheelMotor.setVoltage(voltage);
  }

  /**
   * @return output voltage
   */
  public double colorWheelGetVoltage() {
    return colorWheelMotor.getMotorOutputVoltage();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateColorWheelLog(false);

    // ColorSensor updated values on dashboard
    SmartDashboard.putBoolean("Yellow", colorSensor.getColor() == "Yellow");
    SmartDashboard.putBoolean("Red", colorSensor.getColor() == "Red");
    SmartDashboard.putBoolean("Green", colorSensor.getColor() == "Green");
    SmartDashboard.putBoolean("Blue", colorSensor.getColor() == "Blue");
    SmartDashboard.putString("Color", colorSensor.getColor());
    SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
  }

  /**
   * Write information about color wheel to filelog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  public void updateColorWheelLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, "ColorWheel", "updates", 
      "ColorWheel Volts", colorWheelMotor.getMotorOutputVoltage(), 
      "ColorWheel Amps", colorWheelMotor.getSupplyCurrent()
    );
  }}
