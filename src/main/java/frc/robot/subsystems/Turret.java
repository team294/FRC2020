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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.FileLog;

public class Turret extends SubsystemBase {
  private final WPI_TalonFX turretMotor = new WPI_TalonFX(Constants.TurretConstants.turretPort);
  private FileLog log; // reference to the fileLog
  
  private final int timeoutMs = 30;
  private final double kP, kI, kD, kFF, kMaxOutput, kMinOutput;
  private double currentAngle, targetAngle;

  /**
   * Creates a new Turret.
   */
  public Turret(FileLog log) {
    this.log = log; // save reference to the fileLog

    turretMotor.configFactoryDefault();
    turretMotor.setInverted(false);
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.configClosedloopRamp(0.1); // seconds from neutral to full
    turretMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs);
    turretMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);

    // PID coefficients
    kP = 0;
    kI = 0;
    kD = 0;
    kFF = 0;
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    turretMotor.config_kP(0, kP, timeoutMs);
    turretMotor.config_kI(0, kI, timeoutMs);
    turretMotor.config_kD(0, kD, timeoutMs);
    turretMotor.config_kF(0, kFF, timeoutMs);
    turretMotor.configPeakOutputForward(kMaxOutput);
    turretMotor.configPeakOutputReverse(kMinOutput);
  }

  /**
   * @param angle angle to move to, in degrees
   */
  public void turretSetAngle(double angle) {
    turretMotor.set(ControlMode.Position, degreesToEncoderTicks(angle), DemandType.Neutral, kFF);
  }

  /**
   * @param percent percent output (0 to 1)
   */
  public void turretSetPercentOutput(double percent) {
    turretMotor.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Set current angle to zero
   */
  public void turretZeroAngle() {
    currentAngle = 0;
  }

  /**
   * @return current turret angle, in degrees
   */
  public double turretGetAngle() {
    return currentAngle;
  }

  /**
   * @return target angle, in degrees
   */
  public double turretGetTargetAngle() {
    return targetAngle;
  }

  /**
   * @return angle error, in degrees
   */
  public double turretGetAngleError() {
    return encoderTicksToDegrees(turretMotor.getClosedLoopError());
  }

  /**
   * @param encoderTicks in encoder ticks
   * @return parameter encoder ticks converted to equivalent degrees
   */
  public double encoderTicksToDegrees(int encoderTicks) {
    return encoderTicks / Constants.TurretConstants.ticksPerDegree;
  }

  /**
   * @param degrees in degrees
   * @return parameter degrees converted to equivalent encoder ticks
   */
  public double degreesToEncoderTicks(double degrees) {
    return degrees * Constants.TurretConstants.ticksPerDegree;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentAngle = encoderTicksToDegrees(turretMotor.getSelectedSensorPosition());

    updateTurretLog(false);
  }

  /**
   * Write information about turret to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
	public void updateTurretLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, "Turret", "Update Variables",  
      "Motor Voltage", turretMotor.getMotorOutputVoltage(),
      "Motor Amps", turretMotor.getSupplyCurrent(),
      "Current Angle", currentAngle,
      "Angle Error", turretGetAngleError()
    );
  }
}
