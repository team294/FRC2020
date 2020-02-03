/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.*;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class Climb extends SubsystemBase {
  private final WPI_TalonFX climbMotor1;
  private final WPI_TalonFX climbMotor2;
  private final Solenoid climbPiston1;
  private final Solenoid climbPiston2;

  private FileLog log;

  private double measuredVelocityRaw, measuredRPM, feederRPM, setPoint;
  private double kP, kI, kD, kFF, kMaxOutput, kMinOutput; // PID terms
  private int timeoutMs = 30;
  private double ticksPer100ms = 600.0 / 2048.0; // convert raw units to RPM (2048 ticks per revolution)
  private double ff, p, i, d; // for shuffleboard

  public Climb(FileLog log) {
    climbMotor1 = new WPI_TalonFX(ClimbConstants.climbMotorPort1);
    climbMotor2 = new WPI_TalonFX(ClimbConstants.climbMotorPort2);
    climbPiston1 = new Solenoid(ClimbConstants.climbPistonPort1);
    climbPiston2 = new Solenoid(ClimbConstants.climbPistonPort2);

    this.log = log;

    climbMotor1.configFactoryDefault();
    climbMotor2.configFactoryDefault();

    climbMotor1.setInverted(true);
    climbMotor2.setInverted(false);

    climbMotor1.setNeutralMode(NeutralMode.Brake);
    climbMotor1.configClosedloopRamp(0.05); // seconds from neutral to full
    climbMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs); 
    climbMotor1.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);

    climbMotor2.setNeutralMode(NeutralMode.Brake);
    climbMotor2.configClosedloopRamp(0.05); // seconds from neutral to full
    climbMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs); 
    climbMotor2.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    
    // PID coefficients
    kP = 0;
    kI = 0;
    kD = 0; 
    kFF = 0;
    
    // set climb1 PID coefficients
    climbMotor1.config_kF(0, kFF, timeoutMs);
    climbMotor1.config_kP(0, kP, timeoutMs);
    climbMotor1.config_kI(0, kI, timeoutMs);
    climbMotor1.config_kD(0, kD, timeoutMs);

    // set climb2 PID coefficients
    climbMotor2.config_kF(0, kFF, timeoutMs);
    climbMotor2.config_kP(0, kP, timeoutMs);
    climbMotor2.config_kI(0, kI, timeoutMs);
    climbMotor2.config_kD(0, kD, timeoutMs);

    kMaxOutput = 1; 
    kMinOutput = -1;

    climbMotor1.configPeakOutputForward(kMaxOutput);
    climbMotor1.configPeakOutputReverse(kMinOutput);
    climbMotor1.setSensorPhase(false);

    climbMotor2.configPeakOutputForward(kMaxOutput);
    climbMotor2.configPeakOutputReverse(kMinOutput);
    climbMotor2.setSensorPhase(false);
  }

  /**
   * @param extend true = extend, false = retract
   */
  public void climbPistonsSetPosition(boolean extend) {
    climbPiston1.set(extend);
    climbPiston2.set(extend);
  }

  /**
   * @return true = extended, false = retracted
   */
  public boolean climbPistonsGetPosition() {
    return climbPiston1.get();
  }

  /**
   * @param percent percent output (0 to 1)
   */
  public void climbMotorsSetPercentOutput(double percent) {
    climbMotor1SetPercentOutput(percent);
    climbMotor2SetPercentOutput(percent);
  }

  /**
   * @param inches position, inches
   */
  public void climbMotorsSetPosition(double inches) {
    climbMotor1SetPosition(inches);
    climbMotor2SetPosition(inches);
  }

  /**
   * @param percent percent output (0 to 1)
   */
  public void climbMotor1SetPercentOutput(double percent) {
    climbMotor1.set(ControlMode.PercentOutput, percent);
  }

  /**
   * @param percent percent output (0 to 1)
   */
  public void climbMotor2SetPercentOutput(double percent) {
    climbMotor2.set(ControlMode.PercentOutput, percent);
  }

  /**
   * @param inches position, inches
   */
  public void climbMotor1SetPosition(double inches) {
    climbMotor1.set(ControlMode.Position, inchesToEncoderTicks(inches), DemandType.ArbitraryFeedForward, kFF);
  }

  /**
   * @param inches position, inches
   */
  public void climbMotor2SetPosition(double inches) {
    climbMotor2.set(ControlMode.Position, inchesToEncoderTicks(inches), DemandType.ArbitraryFeedForward, kFF);
  }

  /**
   * @param ticks encoder ticks
   * @return parameter encoder ticks converted to equivalent inches
   */
  public double encoderTicksToInch(double ticks) {
    return ticks / Constants.ClimbConstants.ticksPerInch;
  }

  /**
   * @param inches inches
   * @return parameter inches converted to equivalent encoder ticks
   */
  public double inchesToEncoderTicks(double inches) {
    return inches * Constants.ClimbConstants.ticksPerInch;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateClimbLog(false);
  }

  /**
   * Write information about climb to filelog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  public void updateClimbLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, "Climb", "Update Variables",  
      "Motor1 Volt", climbMotor1.getMotorOutputVoltage(), 
      "Motor2 Volt", climbMotor2.getMotorOutputVoltage(),
      "Motor1 Amps", climbMotor1.getSupplyCurrent(),
      "Motor2 Amps", climbMotor2.getSupplyCurrent()
    );
  }
}
