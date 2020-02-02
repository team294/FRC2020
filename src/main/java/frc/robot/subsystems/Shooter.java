/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.FileLog;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX shooterMotor1 = new WPI_TalonFX(Constants.ShooterConstants.shooter1Port);
  private final WPI_TalonFX shooterMotor2 = new WPI_TalonFX(Constants.ShooterConstants.shooter2Port);
  private FileLog log; // reference to the fileLog

  private double measuredVelocityRaw, measuredRPM, shooterRPM, setPoint;
  private double kP, kI, kD, kFF, kMaxOutput, kMinOutput; // PID terms
  private int timeoutMs = 0; // was 30, changed to 0 for testing
  private double ticksPer100ms = 600.0 / 2048.0; // convert raw units to RPM (2048 ticks per revolution)
  
  public Shooter(FileLog log) {
    this.log = log; // save reference to the fileLog

    shooterMotor1.configFactoryDefault();
    shooterMotor2.configFactoryDefault();
    shooterMotor1.setInverted(true);
    shooterMotor2.setInverted(false);

    // set drives to coast mode and ramp rate
    shooterMotor1.setNeutralMode(NeutralMode.Coast);
    shooterMotor2.setNeutralMode(NeutralMode.Coast);
    shooterMotor1.configClosedloopRamp(0.05); //seconds from neutral to full
    shooterMotor2.configClosedloopRamp(0.05);
    shooterMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs); 
    // shooterMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs);
    shooterMotor1.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    
    // shooterMotor2.set(ControlMode.Follower, Constants.ShooterConstants.shooter1Port);     Redundant with line 67
    
    // PID coefficients initial
    kP = 0.25;
    kI = 0;
    kD = 0; 
    kFF = 0.052; // 0.052 empirically to set to 3000 RPM on prototype shooter (one motor)
    
    // set PID coefficients
    shooterMotor1.config_kF(0, kFF, timeoutMs);
    shooterMotor1.config_kP(0, kP, timeoutMs);
    shooterMotor1.config_kI(0, kI, timeoutMs);
    shooterMotor1.config_kD(0, kD, timeoutMs);

    kMaxOutput = 1; 
    kMinOutput = 0; // no need to go negative, a negative number will brake (and possibly break) the motor
    
    shooterMotor1.configPeakOutputForward(kMaxOutput);
    shooterMotor1.configPeakOutputReverse(kMinOutput);
    shooterMotor1.setSensorPhase(false);

    shooterMotor2.set(ControlMode.Follower,Constants.ShooterConstants.shooter1Port);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Shooter P", kP);
    SmartDashboard.putNumber("Shooter I", kI);
    SmartDashboard.putNumber("Shooter D", kD);
    SmartDashboard.putNumber("Shooter FF", kFF);
    
    SmartDashboard.putNumber("Shooter SetPoint RPM", shooterRPM);
    SmartDashboard.putNumber("Shooter Manual SetPoint RPM", shooterRPM);
  }

  /**
   * @param voltage voltage
   */
  public void setShooterVoltage(double voltage) {
    shooterMotor1.setVoltage(voltage);
  }

  /**
   * Run shooter in a velocity closed loop mode.
   * If setPoint RPM is 0, this uses Shuffleboard as input.
   * @param shooterRPM setPoint RPM
   */
  public void setShooterPID(double shooterRPM) {
    this.shooterRPM = shooterRPM;
    setPoint = shooterRPM / ticksPer100ms; // setPoint is in ticks per 100ms
    shooterMotor1.set(ControlMode.Velocity, setPoint);
    SmartDashboard.putNumber("Shooter SetPoint RPM", shooterRPM );
    
    System.out.println("Starting setShooterPID");
  }

  /**
   * @return PID error, in RPM
   */
  public double getShooterPIDError() {
    return shooterMotor1.getClosedLoopError() * ticksPer100ms;
  }

  public double getMeasuredRPM() {
    measuredVelocityRaw = shooterMotor1.getSelectedSensorVelocity(0);
    measuredRPM = measuredVelocityRaw * ticksPer100ms; // converts ticks per 100ms to RPM
    return measuredRPM;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShooterLog(false);

    // read PID coefficients from SmartDashboard
    double ff = SmartDashboard.getNumber("Shooter FF", 0);
    double p = SmartDashboard.getNumber("Shooter P", 0);
    double i = SmartDashboard.getNumber("Shooter I", 0);
    double d = SmartDashboard.getNumber("Shooter D", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if(ff != kFF) shooterMotor1.config_kF(0, ff, timeoutMs); kFF = ff;
    if(p != kP) shooterMotor1.config_kP(0, p, timeoutMs); kP = p;
    if(i != kI) shooterMotor1.config_kI(0, i, timeoutMs); kI = i;
    if(d != kD) shooterMotor1.config_kD(0, d, timeoutMs); kD = d;
    
    measuredRPM = getMeasuredRPM();
    
    SmartDashboard.putNumber("Shooter SetPoint RPM", setPoint * ticksPer100ms);
    SmartDashboard.putNumber("Shooter RPM", measuredRPM);
    SmartDashboard.putNumber("Shooter Motor 1 Current", shooterMotor1.getSupplyCurrent());
    SmartDashboard.putNumber("Shooter Motor 2 Current", shooterMotor2.getSupplyCurrent());
    SmartDashboard.putNumber("Shooter PID Error", getShooterPIDError());
    SmartDashboard.putNumber("Shooter PercentOutput", shooterMotor1.getMotorOutputPercent());
  }

  /**
   * Write information about shooter to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
	public void updateShooterLog(boolean logWhenDisabled) {
		log.writeLog(logWhenDisabled, "Shooter", "Update Variables",  
      "Motor RPM", shooterMotor1.getSelectedSensorVelocity(0) *  ticksPer100ms,
      "Motor Volt", shooterMotor1.getMotorOutputVoltage(), 
      "Motor1 Amps", shooterMotor1.getSupplyCurrent(),
      "Motor2 Amps", shooterMotor2.getSupplyCurrent(),
      "Measured RPM", measuredRPM,
      "PID Error", getShooterPIDError()
    );
  }
}
