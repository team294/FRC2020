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

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TemperatureCheck;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX shooterMotorLeft = new WPI_TalonFX(canShooterMotorLeft);
  private final WPI_TalonFX shooterMotorRight = new WPI_TalonFX(canShooterMotorRight);
  private final Solenoid shooterHoodPiston = new Solenoid(pcmShooterHoodPiston); // piston to open and close hood
  private final Solenoid shooterLockPiston = new Solenoid(pcmShooterLockPiston); // piston to lock hood angle
  private FileLog log; // reference to the fileLog
  private TemperatureCheck tempCheck;
  private Hopper hopper;

  private double measuredVelocityRaw, measuredRPM, shooterRPM, setPoint, voltageTarget = 1; // setPoint is in native units
  private double kP, kI, kD, kFF, kMaxOutput, kMinOutput; // PID terms
  private int timeoutMs = 0; // was 30, changed to 0 for testing
  private double ticksPer100ms = 600.0 / 2048.0; // convert raw units to RPM (2048 ticks per revolution)
  private int powerCellsShot = 0;
  private double prevVoltage = 0;
  
  public Shooter(Hopper hopper, FileLog log, TemperatureCheck tempCheck) {
    this.log = log; // save reference to the fileLog
    this.tempCheck = tempCheck;
    this.hopper = hopper;

    setLockPiston(false);

    shooterMotorLeft.configFactoryDefault();
    shooterMotorRight.configFactoryDefault();
    shooterMotorLeft.setInverted(true);
    shooterMotorRight.setInverted(false);

    // set drives to coast mode and ramp rate
    shooterMotorLeft.setNeutralMode(NeutralMode.Coast);
    shooterMotorRight.setNeutralMode(NeutralMode.Coast);
    shooterMotorLeft.configClosedloopRamp(0.05); //seconds from neutral to full
    shooterMotorRight.configClosedloopRamp(0.05);
    shooterMotorLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs); 
    // shooterMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs);
    shooterMotorLeft.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        
    // PID coefficients initial
    kP = 0.25;
    kI = 0;
    kD = 0; 
    kFF = 0.052; // 0.052 empirically to set to 3000 RPM on prototype shooter (one motor)
    
    // set PID coefficients
    shooterMotorLeft.config_kF(0, kFF, timeoutMs);
    shooterMotorLeft.config_kP(0, kP, timeoutMs);
    shooterMotorLeft.config_kI(0, kI, timeoutMs);
    shooterMotorLeft.config_kD(0, kD, timeoutMs);

    kMaxOutput = 1; 
    kMinOutput = 0; // no need to go negative, a negative number will brake (and possibly break) the motor
    
    shooterMotorLeft.configPeakOutputForward(kMaxOutput);
    shooterMotorLeft.configPeakOutputReverse(kMinOutput);
    shooterMotorLeft.setSensorPhase(false);

    shooterMotorRight.set(ControlMode.Follower, canShooterMotorLeft);

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
    shooterMotorLeft.setVoltage(voltage);
    voltageTarget = voltage;
  }

  /**
   * Run shooter in a velocity closed loop mode.
   * If setPoint RPM is 0, this uses Shuffleboard as input.
   * @param shooterRPM setPoint RPM
   */
  public void setShooterPID(double shooterRPM) {
    this.shooterRPM = shooterRPM;
    setPoint = shooterRPM / ticksPer100ms; // setPoint is in ticks per 100ms
    shooterMotorLeft.set(ControlMode.Velocity, setPoint);
    voltageTarget = 1;
    SmartDashboard.putNumber("Shooter SetPoint RPM", shooterRPM );
    System.out.println("Starting setShooterPID");
  }

  /**
   * @param retract true = retract (open), false = extend (close)
   */
  public void setHoodPiston(boolean retract) {
    shooterHoodPiston.set(retract);
  }

  /**
   * @param retract true = retract (unlock), false = extend (lock)
   */
  public void setLockPiston(boolean retract) {
    shooterLockPiston.set(retract);
  }

  /**
   * @return PID error, in RPM
   */
  public double getShooterPIDError() {
    return shooterMotorLeft.getClosedLoopError() * ticksPer100ms;
  }

  /**
   * @return measured rpm
   */
  public double getMeasuredRPM() {
    measuredVelocityRaw = shooterMotorLeft.getSelectedSensorVelocity(0);
    measuredRPM = measuredVelocityRaw * ticksPer100ms; // converts ticks per 100ms to RPM
    return measuredRPM;
  }

  /**
   * @return output voltage
   */
  public double getVoltage() {
    return shooterMotorLeft.getMotorOutputVoltage();
  }

  /**
   * @return power cells shot
   */
  public int getPowerCellsShot() {
    return powerCellsShot;
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
    if(ff != kFF) shooterMotorLeft.config_kF(0, ff, timeoutMs); kFF = ff;
    if(p != kP) shooterMotorLeft.config_kP(0, p, timeoutMs); kP = p;
    if(i != kI) shooterMotorLeft.config_kI(0, i, timeoutMs); kI = i;
    if(d != kD) shooterMotorLeft.config_kD(0, d, timeoutMs); kD = d;
    
    measuredRPM = getMeasuredRPM();
    
    SmartDashboard.putNumber("Shooter SetPoint RPM", setPoint * ticksPer100ms);
    SmartDashboard.putNumber("Shooter RPM", measuredRPM);
    SmartDashboard.putNumber("Shooter Motor 1 Current", shooterMotorLeft.getSupplyCurrent());
    SmartDashboard.putNumber("Shooter Motor 2 Current", shooterMotorRight.getSupplyCurrent());
    SmartDashboard.putNumber("Shooter PID Error", getShooterPIDError());
    SmartDashboard.putNumber("Shooter PercentOutput", shooterMotorLeft.getMotorOutputPercent());
    SmartDashboard.putNumber("Shooter Voltage", shooterMotorLeft.getMotorOutputVoltage());
    SmartDashboard.putNumber("Power Cells Shot", powerCellsShot);
    
    if(log.getLogRotation() == log.SHOOTER_CYCLE) {
      updateShooterLog(false);
    }

    if (getVoltage() > voltageCheck && prevVoltage < voltageCheck && Math.abs(hopper.hopperGetPercentOutput()) > hopperPercentCheck)
      powerCellsShot++;
    if (voltageTarget == 0) powerCellsShot = 0;

    prevVoltage = getVoltage();
  }

  /**
   * Write information about shooter to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
	public void updateShooterLog(boolean logWhenDisabled) {
		log.writeLog(logWhenDisabled, "Shooter", "Update Variables",  
      //"Motor RPM", shooterMotorLeft.getSelectedSensorVelocity(0) *  ticksPer100ms,  Same as measuredRPM
      "Motor Volt", shooterMotorLeft.getMotorOutputVoltage(), 
      "Left Motor Amps", shooterMotorLeft.getSupplyCurrent(),
      // "Left Temp", shooterMotorRight.getTemperature(),
      "Right Motor Amps", shooterMotorRight.getSupplyCurrent(),
      // "Right Temp", shooterMotorRight.getTemperature(),
      "Measured RPM", measuredRPM,
      "PID Error", getShooterPIDError()
    );
  }

  /**
   * Update TemperatureCheck utility with motors that are and are not overheating.
   */
  public void updateOverheatingMotors() {
    if (shooterMotorLeft.getTemperature() >= temperatureCheck)
      tempCheck.recordOverheatingMotor("ShooterLeft");
    if (shooterMotorRight.getTemperature() >= temperatureCheck)
      tempCheck.recordOverheatingMotor("ShooterRight");

    if (shooterMotorLeft.getTemperature() < temperatureCheck)
      tempCheck.notOverheatingMotor("ShooterLeft");
    if (shooterMotorRight.getTemperature() < temperatureCheck)
      tempCheck.notOverheatingMotor("ShooterRight");
  }
}
