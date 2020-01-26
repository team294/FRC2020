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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  
  private final WPI_TalonFX shooterMotor1 = new WPI_TalonFX(Constants.ShooterConstants.shooter1Port); 
  private final WPI_TalonFX shooterMotor2 = new WPI_TalonFX(Constants.ShooterConstants.shooter2Port); 
  private double measuredVelocityRaw, measuredRPM;
  private double kP, kI, kD, kFF, kMaxOutput, kMinOutput, shooterRPM, setPoint;
  private int kTimeoutMs = 0; // was 30, changed to 0 for testing
  private double ticksPer100ms = 600.0 / 2048.0; // convert raw units to RPM (2048 ticks per revolution)
  
  public Shooter() {
    shooterMotor1.configFactoryDefault();
    shooterMotor2.configFactoryDefault();
    shooterMotor1.setInverted(true);
    shooterMotor2.setInverted(false);

    // set drives to coast mode and ramp rate
    shooterMotor1.setNeutralMode(NeutralMode.Coast);
    shooterMotor2.setNeutralMode(NeutralMode.Coast);
    shooterMotor1.configClosedloopRamp(0.05); //seconds from neutral to full
    shooterMotor2.configClosedloopRamp(0.05);
    shooterMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeoutMs); 
    shooterMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeoutMs); 

    shooterMotor2.set(ControlMode.Follower, Constants.ShooterConstants.shooter1Port);
    
    // PID coefficients initial
    kP = 0.25;
    kI = 0;
    kD = 0; 
    kFF = 0.052; // 0.052 empirically to set to 3000 RPM on prototype shooter one motor
    
    // set PID coefficients
    shooterMotor1.config_kF(0, kFF, kTimeoutMs);
    shooterMotor1.config_kP(0, kP, kTimeoutMs);
    shooterMotor1.config_kI(0, kI, kTimeoutMs);
    shooterMotor1.config_kD(0, kD, kTimeoutMs);

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
   * This runs the Shooter in a velocity closed loop mode.
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
   * @return PID error, in rpm
   */
  public double getShooterPIDError() {
    return shooterMotor1.getClosedLoopError() / ticksPer100ms;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // read PID coefficients from SmartDashboard
    double ff = SmartDashboard.getNumber("Shooter FF", 0);
    double p = SmartDashboard.getNumber("Shooter P", 0);
    double i = SmartDashboard.getNumber("Shooter I", 0);
    double d = SmartDashboard.getNumber("Shooter D", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if(ff != kFF) shooterMotor1.config_kF(0, ff, kTimeoutMs); kFF = ff;
    if(p != kP) shooterMotor1.config_kP(0, p, kTimeoutMs); kP = p;
    if(i != kI) shooterMotor1.config_kI(0, i, kTimeoutMs); kI = i;
    if(d != kD) shooterMotor1.config_kD(0, d, kTimeoutMs); kD = d;
    
    measuredVelocityRaw = shooterMotor1.getSelectedSensorVelocity(0);
    measuredRPM = measuredVelocityRaw * ticksPer100ms; // converts ticks per 100ms to RPM
    
    SmartDashboard.putNumber("Shooter SetPoint RPM",  setPoint * ticksPer100ms);
    SmartDashboard.putNumber("Shooter RPM", measuredRPM);
    SmartDashboard.putNumber("Shooter Motor 1 Current",shooterMotor1.getSupplyCurrent());
    SmartDashboard.putNumber("Shooter Motor 2 Current",shooterMotor2.getSupplyCurrent());

  }
}
