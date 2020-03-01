/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;
import frc.robot.Constants.RobotConstants;

import static frc.robot.Constants.IntakeConstants.*;


public class Intake extends SubsystemBase {
  private final BaseMotorController intakeMotor;
  private static double intakeCurrent = 0;

  private final DoubleSolenoid intakePiston = new DoubleSolenoid(pcmIntakePistonOut, pcmIntakePistonIn);
  private FileLog log; 
 
  public Intake(FileLog log) {
    this.log = log;

    if (RobotConstants.prototypeBot) { 
      intakeMotor = new WPI_VictorSPX(canIntakeMotor);
    }
    else {
      intakeMotor = new WPI_TalonSRX(canIntakeMotor);
    }
    intakeMotor.configFactoryDefault();
    intakeMotor.setInverted(true);
    intakeMotor.setNeutralMode(NeutralMode.Brake);

    
  }

  /**
   * @param percent percent output (0 to 1)
   */
  public void intakeSetPercentOutput(double percent) {
    intakeMotor.set(ControlMode.PercentOutput, percent);
  }

  /**
   * @param extend true = extend, false = retract
   */
  public void intakeSetPiston(boolean extend) {
    if (extend) intakePiston.set(Value.kForward);
    else if (!extend) intakePiston.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(log.getLogRotation() == log.INTAKE_CYCLE) {
      if (!RobotConstants.prototypeBot) {
        intakeCurrent = ((WPI_TalonSRX) intakeMotor).getSupplyCurrent();
      } else {
        intakeCurrent = 0;  
      } 

      updateIntakeLog(false);
      
      SmartDashboard.putNumber("Intake % Output", intakeMotor.getMotorOutputPercent());
      SmartDashboard.putNumber("Intake Voltage", intakeMotor.getMotorOutputVoltage()); 
      SmartDashboard.putNumber("Intake Current", intakeCurrent);
      SmartDashboard.putNumber("Intake Percent", intakeMotor.getMotorOutputPercent());
    }
  }

  /**
   * Write information about intake to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
	public void updateIntakeLog(boolean logWhenDisabled) {
		log.writeLog(logWhenDisabled, "Intake", "Update Variables",  
      "Motor Volt", intakeMotor.getMotorOutputVoltage(), 
      "Motor Output %", intakeMotor.getMotorOutputPercent()
    );
  }
}
