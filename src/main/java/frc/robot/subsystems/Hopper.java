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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.HopperConstants.*;

public class Hopper extends SubsystemBase {
  private final WPI_VictorSPX hopperMotor = new WPI_VictorSPX(canHopperMotor);
  
  public Hopper() {
    hopperMotor.configFactoryDefault();
    hopperMotor.setInverted(false);
    hopperMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * @param percent percent output (0 to 1)
   */
  public void hopperSetPercentOutput(double percent) {
    hopperMotor.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
