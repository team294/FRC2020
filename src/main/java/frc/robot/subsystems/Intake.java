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
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(Constants.IntakeConstants.intakeMotor);
 
  public Intake() {
    intakeMotor.configFactoryDefault();
    intakeMotor.setInverted(false);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * @param percent percent output (0 to 1)
   */
  public void intakeSetPercentOutput(double percent) {
    intakeMotor.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
