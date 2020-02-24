/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;

import static frc.robot.Constants.HopperConstants.*;

public class Hopper extends SubsystemBase {
  private final WPI_VictorSPX hopperMotor = new WPI_VictorSPX(canHopperMotor);

  private FileLog log;
  
  public Hopper(FileLog log) {
    this.log = log;

    hopperMotor.configFactoryDefault();
    hopperMotor.setInverted(true);
    hopperMotor.setNeutralMode(NeutralMode.Brake);
    hopperMotor.configOpenloopRamp(0.15); // seconds from neutral to full
  }

  /**
   * @param percent percent output (0 to 1)
   */
  public void hopperSetPercentOutput(double percent) {
    hopperMotor.set(ControlMode.PercentOutput, percent);
  }

  /**
   * @return motor percent output (0 to 1)
   */
  public double hopperGetPercentOutput() {
    return hopperMotor.getMotorOutputPercent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(log.getLogRotation() == log.HOPPER_CYCLE) {
      updateHopperLog(false);

      SmartDashboard.putNumber("Hopper % Output", hopperMotor.getMotorOutputPercent());
      SmartDashboard.putNumber("Hopper Voltage", hopperMotor.getMotorOutputVoltage());
    }
  }

  /**
   * Write information about hopper to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
	public void updateHopperLog(boolean logWhenDisabled) {
		log.writeLog(logWhenDisabled, "Hopper", "Update Variables",  
      "Motor Volt", hopperMotor.getMotorOutputVoltage(), 
      "Motor Output %", hopperMotor.getMotorOutputPercent()
    );
  }
}
