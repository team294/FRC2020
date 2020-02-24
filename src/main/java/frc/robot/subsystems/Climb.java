/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.*;

import static frc.robot.Constants.ClimbConstants.*;

public class Climb extends SubsystemBase {
  private final WPI_TalonFX climbMotorLeft = new WPI_TalonFX(canClimbMotorLeft); // 50 to 1 gear ratio, 1 inch per second
  private final WPI_TalonFX climbMotorRight = new WPI_TalonFX(canClimbMotorRight);
  private final DoubleSolenoid climbPistons = new DoubleSolenoid(pcmClimbPistonsOut, pcmClimbPistonsIn);
  private final TalonFXSensorCollection leftLimit, rightLimit;

  private FileLog log;

  private double leftTargetVelocity, leftCurrVelocity, rightTargetVelocity, rightCurrVelocity;
  private double kP, kI, kD, kFF, kMaxOutput, kMinOutput; // PID terms
  private int timeoutMs = 30;

  public Climb(FileLog log) {
    this.log = log;

    climbMotorLeft.configFactoryDefault();
    climbMotorRight.configFactoryDefault();

    climbMotorLeft.setInverted(true); // TODO determine motor inversion
    climbMotorRight.setInverted(false);

    climbMotorLeft.setNeutralMode(NeutralMode.Coast);
    climbMotorLeft.configClosedloopRamp(0.05); // seconds from neutral to full
    climbMotorLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs); 
    climbMotorLeft.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);

    climbMotorRight.setNeutralMode(NeutralMode.Brake);
    climbMotorRight.configClosedloopRamp(0.05); // seconds from neutral to full
    climbMotorRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs); 
    climbMotorRight.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);

    leftLimit = climbMotorLeft.getSensorCollection();
    rightLimit = climbMotorRight.getSensorCollection();
    
    // PID coefficients
    kP = 0;
    kI = 0;
    kD = 0; 
    kFF = 0.1;
    
    // set climb1 PID coefficients
    climbMotorLeft.config_kF(0, kFF, timeoutMs);
    climbMotorLeft.config_kP(0, kP, timeoutMs);
    climbMotorLeft.config_kI(0, kI, timeoutMs);
    climbMotorLeft.config_kD(0, kD, timeoutMs);

    // set climb2 PID coefficients
    climbMotorRight.config_kF(0, kFF, timeoutMs);
    climbMotorRight.config_kP(0, kP, timeoutMs);
    climbMotorRight.config_kI(0, kI, timeoutMs);
    climbMotorRight.config_kD(0, kD, timeoutMs);

    kMaxOutput = 1; 
    kMinOutput = -1;

    climbMotorLeft.configPeakOutputForward(kMaxOutput);
    climbMotorLeft.configPeakOutputReverse(kMinOutput);
    climbMotorLeft.setSensorPhase(false);

    climbMotorRight.configPeakOutputForward(kMaxOutput);
    climbMotorRight.configPeakOutputReverse(kMinOutput);
    climbMotorRight.setSensorPhase(false);
  }

  /***************************
   * VELOCITY CONTROL
   ***************************/

   /**
    * @param ips velocity, in inches per second
    * @return parameter inches per second converted to equivalent ticks per 100ms
    */
  public double ipsToTicksPer100ms(double ips) {
    return (ips * ticksPerInch) / 10;
  }

  /**
    * @param ticksPer100ms velocity, in ticks per 100ms
    * @return parameter ticks per 100ms converted to equivalent inches per second
    */
    public double ticksPer100msToIPS(double ticksPer100ms) {
      return (ticksPer100ms / ticksPerInch) * 10;
    }

  /**
   * @param velocity target velocity, in inches per second
   */
  public void climbMotorLeftSetVelocity(double velocity) {
    leftTargetVelocity = velocity;
    climbMotorLeft.set(ControlMode.Velocity, ipsToTicksPer100ms(velocity));
  }

  /**
   * @param velocity target velocity, in inches per second
   */
  public void climbMotorRightSetVelocity(double velocity) {
    rightTargetVelocity = velocity;
    climbMotorLeft.set(ControlMode.Velocity, ipsToTicksPer100ms(velocity));
  }

  /**
   * @param velocity target velocity for both motors, in inches per second
   */
  public void climbMotorsSetVelocity(double velocity) {
    climbMotorLeftSetVelocity(velocity);
    climbMotorRightSetVelocity(velocity);
  }

  /***************************
   * LIMIT SWITCHES
   ***************************/

  /**
   * @return true = left limit switch is triggered, false = not triggered
   */
  public boolean getLeftLimit() {
    if(leftLimit.isRevLimitSwitchClosed() == 1) return true;
    else return false;
  }

  /**
   * @return true = right limit switch is triggered, false = not triggered
   */
  public boolean getRightLimit() {
    if(rightLimit.isRevLimitSwitchClosed() == 1) return true;
    else return false;
  }

  /***************************
   * ENCODER DATA
   ***************************/

  /**
   * @return left arm raw encoder position, in ticks
   */
  public double getLeftEncoderRaw() {
    return climbMotorLeft.getSelectedSensorPosition();
  }

  /**
   * @return right arm raw encoder position, in ticks
   */
  public double getRightEncoderRaw() {
    return climbMotorRight.getSelectedSensorPosition();
  }

  /**
   * Set current position on the left climb arm as zero
   */
  public void zeroLeftEncoder() {
    climbMotorLeft.setSelectedSensorPosition(0);
  }

  /**
   * Set current position on the right climb arm as zero
   */
  public void zeroRightEncoder() {
    climbMotorRight.setSelectedSensorPosition(0);
  }

  /**
   * @return left arm position, in inches
   */
  public double getLeftEncoderInches() {
    return encoderTicksToInch(getLeftEncoderRaw());
  }

  /**
   * @return right arm position, in inches
   */
  public double getRightEncoderInches() {
    return encoderTicksToInch(getRightEncoderRaw());
  }

  /**
   * @param ticks encoder ticks
   * @return parameter encoder ticks converted to equivalent inches
   */
  public double encoderTicksToInch(double ticks) {
    return ticks / ticksPerInch;
  }

  /**
   * @param inches inches
   * @return parameter inches converted to equivalent encoder ticks
   */
  public double inchesToEncoderTicks(double inches) {
    return inches * ticksPerInch;
  }

  /***************************
   * PISTON CONTROL
   ***************************/

  /**
   * @param extend true = extend, false = retract
   */
  public void climbPistonsSetPosition(boolean extend) {
    if (extend) climbPistons.set(Value.kForward);
    else climbPistons.set(Value.kReverse);
  }

  /**
   * @return true = extended, false = retracted
   */
  public boolean climbPistonsGetPosition() {
    if (climbPistons.get() == Value.kForward) return true;
    else return false;
  }

  /***************************
   * PERCENT OUTPUT CONTROL
   ***************************/

  /**
   * @param percent percent output (0 to 1)
   */
  public void climbMotorsSetPercentOutput(double percent) {
    climbMotorLeftSetPercentOutput(percent);
    climbMotorRightSetPercentOutput(percent);
  }

  /**
   * @param percent percent output (0 to 1)
   */
  public void climbMotorLeftSetPercentOutput(double percent) {
    climbMotorLeft.set(ControlMode.PercentOutput, percent);
  }

  /**
   * @param percent percent output (0 to 1)
   */
  public void climbMotorRightSetPercentOutput(double percent) {
    climbMotorRight.set(ControlMode.PercentOutput, percent);
  }

  /***************************
   * POSITION CONTROL
   ***************************/
  
  /**
   * @param inches target position, in inches
   */
  public void climbMotorsSetPosition(double inches) {
    climbMotorLeftSetPosition(inches);
    climbMotorRightSetPosition(inches);
  }

  /**
   * @param inches left arm target position, in inches
   */
  public void climbMotorLeftSetPosition(double inches) {
    climbMotorLeft.set(ControlMode.Position, inchesToEncoderTicks(inches));
  }

  /**
   * @param inches right arm target position, in inches
   */
  public void climbMotorRightSetPosition(double inches) {
    climbMotorRight.set(ControlMode.Position, inchesToEncoderTicks(inches));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(getLeftLimit()) zeroLeftEncoder();
    if(getRightLimit()) zeroRightEncoder();

    leftCurrVelocity = ticksPer100msToIPS(climbMotorLeft.getSelectedSensorVelocity());
    rightCurrVelocity = ticksPer100msToIPS(climbMotorRight.getSelectedSensorVelocity());

    if(log.getLogRotation() == log.CLIMB_CYCLE) {
      updateClimbLog(false);

      SmartDashboard.putBoolean("ClimbLeft Limit", getLeftLimit());
      SmartDashboard.putBoolean("ClimbRight Limit", getRightLimit());
      SmartDashboard.putNumber("ClimbLeft Position", getLeftEncoderInches());
      SmartDashboard.putNumber("ClimbRight Position", getRightEncoderInches());
      SmartDashboard.putNumber("ClimbLeft Vel", leftCurrVelocity);
      SmartDashboard.putNumber("ClimbRight Vel", rightCurrVelocity);
    }
  }

  /**
   * Write information about climb to filelog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  public void updateClimbLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, "Climb", "Update Variables",  
      "Left Volt", climbMotorLeft.getMotorOutputVoltage(), 
      "Right Volt", climbMotorRight.getMotorOutputVoltage(),
      "Left Amps", climbMotorLeft.getSupplyCurrent(),
      "Right Amps", climbMotorRight.getSupplyCurrent(),
      "Left Position", getLeftEncoderInches(),
      "Right Position", getRightEncoderInches(),
      "Left Limit", getLeftLimit(),
      "Right Limit", getRightLimit(),
      "Left Target Vel", leftTargetVelocity,
      "Right Target Vel", rightTargetVelocity,
      "Left Vel", leftCurrVelocity,
      "Right Vel", rightCurrVelocity
    );
  }
}
