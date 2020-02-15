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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.*;
import static frc.robot.Constants.RobotConstants.*;
import static frc.robot.Constants.DriveConstants.*;


public class DriveTrain extends SubsystemBase {
  private final WPI_TalonFX leftMotor1;
  private final WPI_TalonFX leftMotor2;
  private final WPI_TalonFX rightMotor1;
  private final WPI_TalonFX rightMotor2;

  private final DifferentialDrive driveTrain;
  private final DifferentialDriveOdometry odometry;

  private double leftEncoderZero = 0;
  private double rightEncoderZero = 0;

  private final AHRS ahrs;
  private double yawZero = 0;

  private Timer autoTimer;
  private FileLog log;
  private TemperatureCheck tempCheck;
  
  // variables to help calculate angular velocity for turnGyro
  private double prevAng; // last recorded gyro angle
  private double currAng; // current recorded gyro angle
  private double prevTime; // last time gyro angle was recorded
  private double currTime; // current time gyro angle is being recorded
  private double angularVelocity;  // Robot angular velocity in degrees per second
  private LinearFilter lfRunningAvg = LinearFilter.movingAverage(4); //calculate running average to smooth quantization error in angular velocity calc

  
  public DriveTrain(FileLog log, TemperatureCheck tempCheck) {
    this.log = log; // save reference to the fileLog
    this.tempCheck = tempCheck;

    // configure navX
    AHRS gyro = null;
		try {
      gyro = new AHRS(I2C.Port.kMXP);
      gyro.zeroYaw();
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
    ahrs = gyro;
    
    // configure motors
    leftMotor1 = new WPI_TalonFX(canLeftDriveMotor1);
    leftMotor2 = new WPI_TalonFX(canLeftDriveMotor2);
    rightMotor1 = new WPI_TalonFX(canRightDriveMotor1);
    rightMotor2 = new WPI_TalonFX(canRightDriveMotor2);

    leftMotor1.configFactoryDefault();
    leftMotor2.configFactoryDefault();
    rightMotor1.configFactoryDefault();
    rightMotor2.configFactoryDefault();

    leftMotor2.set(ControlMode.Follower, canLeftDriveMotor1);
    rightMotor2.set(ControlMode.Follower, canRightDriveMotor1);

    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);

    // Drive train is reversed on competition robot
    if (prototypeBot) {
      leftMotor1.setInverted(true);
      leftMotor2.setInverted(true);
      rightMotor1.setInverted(true);
      rightMotor2.setInverted(true);
    } else { 
      leftMotor1.setInverted(false);
      leftMotor2.setInverted(false);
      rightMotor1.setInverted(false);
      rightMotor2.setInverted(false);
    }

    setDriveModeCoast(false);

    leftMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rightMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    
    leftMotor1.setSensorPhase(false);
    rightMotor1.setSensorPhase(false);

    leftMotor1.configVoltageCompSaturation(12.0);
    leftMotor2.configVoltageCompSaturation(12.0);
    rightMotor1.configVoltageCompSaturation(12.0);
    rightMotor2.configVoltageCompSaturation(12.0);

    setVoltageCompensation(true);

    // create the drive train AFTER configuring the motors
    driveTrain = new DifferentialDrive(leftMotor1, rightMotor1);
    driveTrain.setDeadband(0.05);
    
    zeroLeftEncoder();
    zeroRightEncoder();
    zeroGyroRotation();

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getGyroRotation()));

    // initialize angular velocity variables
    prevAng = getGyroRaw();
    currAng = getGyroRaw();
    prevTime = System.currentTimeMillis();
    currTime = System.currentTimeMillis();
    lfRunningAvg.reset();
  }

  /**
   * Tank drive method for differential drive platform. The calculated 
   * values will be squared to decrease sensitivity at low speeds.
   * @param leftPercent The robot's left side percent along the X axis [-1.0..1.0]. Forward is positive.
   * @param rightPercent The robot's right side percent along the X axis [-1.0..1.0]. Forward is positive.
   */
  public void tankDrive(double leftPercent, double rightPercent) {
    driveTrain.tankDrive(leftPercent, rightPercent, true);
  }

  /**
   * Tank drive method for differential drive platform.
   * @param leftPercent  The robot's left side percent along the X axis [-1.0..1.0]. Forward is positive.
   * @param rightPercent The robot's right side percent along the X axis [-1.0..1.0]. Forward is positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  public void tankDrive(double leftPercent, double rightPercent, boolean squareInputs) {
    driveTrain.tankDrive(leftPercent, rightPercent, squareInputs);
  }

  /**
   * Call when not using arcade drive or tank drive to turn motors to
   * ensure that motor will not cut out due to differential drive safety.
   */
  public void feedTheDog() {
    driveTrain.feed();
  }

  /**
   * @param percent percent output (+1 = forward, -1 = reverse)
   */
  public void setLeftMotorOutput(double percent) {
    leftMotor1.set(ControlMode.PercentOutput, percent);
    feedTheDog();
  }

  /**
   * @param percent percent output (+1 = forward, -1 = reverse)
   */
  public void setRightMotorOutput(double percent) {
    rightMotor1.set(ControlMode.PercentOutput, -percent);
    feedTheDog();
  }

  public void arcadeDrive(double speedPct, double rotation) {
    driveTrain.arcadeDrive(speedPct, rotation * 0.7, false);    // minimize how fast turn operated from joystick
  }

  /**
   * @param turnOn true = turn on voltage compensation, false = don't turn on voltage compensation
   */
  public void setVoltageCompensation(boolean turnOn) {
    leftMotor1.enableVoltageCompensation(turnOn);
    leftMotor2.enableVoltageCompensation(turnOn);
    rightMotor1.enableVoltageCompensation(turnOn);
    rightMotor2.enableVoltageCompensation(turnOn);
  }

  /**
   * @return left encoder position, in ticks
   */
  public double getLeftEncoderRaw() {
    return leftMotor1.getSelectedSensorPosition(0);
  }

  /**
   * @return right encoder position, in ticks
   */
  public double getRightEncoderRaw() {
    return rightMotor1.getSelectedSensorPosition(0);
  }

  /**
   * @return left encoder velocity, in ticks per 100ms (+ = forward)
   */
  public double getLeftEncoderVelocityRaw() {
    return leftMotor1.getSelectedSensorVelocity(0);
  }

  /**
   * @return right encoder velocity, in ticks per 100ms (+ = forward)
   */
  public double getRightEncoderVelocityRaw() {
    return -rightMotor1.getSelectedSensorVelocity(0);
  }

  /**
   * @param setCoast true = coast mode, false = brake mode
   */
  public void setDriveModeCoast(boolean setCoast) {
    if (setCoast) {
      leftMotor1.setNeutralMode(NeutralMode.Coast);
      leftMotor2.setNeutralMode(NeutralMode.Coast);
      rightMotor1.setNeutralMode(NeutralMode.Coast);
      rightMotor2.setNeutralMode(NeutralMode.Coast);
    } else {
      leftMotor1.setNeutralMode(NeutralMode.Brake);
      leftMotor2.setNeutralMode(NeutralMode.Brake);
      rightMotor1.setNeutralMode(NeutralMode.Brake);
      rightMotor2.setNeutralMode(NeutralMode.Brake);
    }
  }

  /**
	 * Zero the left encoder position in software.
	 */
  public void zeroLeftEncoder() {
    leftEncoderZero = getLeftEncoderRaw();
  }

  /**
	 * Zero the right encoder position in software.
	 */
  public void zeroRightEncoder() {
    rightEncoderZero = getRightEncoderRaw();
  }

  /**
	 * Get the position of the left encoder since last zeroLeftEncoder().
	 * @return encoder position, in ticks
   */
  public double getLeftEncoderTicks() {
    return getLeftEncoderRaw() - leftEncoderZero;
  }

  /**
	 * Get the position of the right encoder since last zeroRightEncoder().
	 * @return encoder position, in ticks
	 */
  public double getRightEncoderTicks() {
    return -(getRightEncoderRaw() - rightEncoderZero);
  }

  /**
   * @param ticks encoder ticks
   * @return parameter encoder ticks converted to equivalent inches
   */
  public static double encoderTicksToInches(double ticks) {
    return ticks / ticksPerInch;
  }

  /**
   * @return left encoder position, in inches
   */
  public double getLeftEncoderInches() {
    return encoderTicksToInches(getLeftEncoderTicks());
  }

  /**
   * @return right encoder position, in inches
   */
  public double getRightEncoderInches() {
    return encoderTicksToInches(getRightEncoderTicks());
  }

  /**
   * @return left encoder velocity, in inches per second (+ = forward)
   */
  public double getLeftEncoderVelocity() {
    return encoderTicksToInches(getLeftEncoderVelocityRaw()) * 10;
  }

  /**
   * @return right encoder velocity, in inches per second (+ = forward)
   */
  public double getRightEncoderVelocity() {
    return encoderTicksToInches(getRightEncoderVelocityRaw()) * 10;
  }

  /**
   * @return average velocity, in inches per second
   */
  public double getAverageEncoderVelocity(){
    return (getRightEncoderVelocity() + getLeftEncoderVelocity()) / 2;
  }

  /**
   * @param inches inches
   * @return parameter inches converted to equivalent encoder ticks
   */
  public double inchesToEncoderTicks(double inches) {
    return inches * ticksPerInch;
  }

  /**
   * Gets the raw gyro angle (can be greater than 360).
   * @return raw gyro angle, in degrees.
   */
  public double getGyroRaw() {
    return ahrs.getAngle();
  }

  /**
	 * Zero the gyro position in software.
	 */
	public void zeroGyroRotation() {
    yawZero = getGyroRaw(); // set yawZero to gyro angle
  }
  
  /**
	 * Resets the gyro position in software to a specified angle.
	 * @param currentHeading gyro heading to reset to, in degrees
	 */
	public void setGyroRotation(double currentHeading) {
		// set yawZero to gryo angle, offset to currentHeading
		yawZero = getGyroRaw() - currentHeading;
		// System.err.println("PLZ Never Zero the Gyro Rotation it is not good");
  }

  /**
	 * @return gyro angle from -180 to 180, in degrees
	 */
	public double getGyroRotation() {
		double angle = getGyroRaw() - yawZero;
		// Angle will be in terms of raw gyro units (-inf,inf), so you need to convert to (-180, 180]
		angle = normalizeAngle(angle);
		return angle;
  }

  public double getAngularVelocity () {
    return angularVelocity;
  }

  /**
	 * Converts input angle to a number between -179.999 and +180.0.
	 * @return normalized angle
	 */
	public double normalizeAngle(double angle) {
		angle = angle % 360;
		angle = (angle <= -180) ? (angle + 360) : angle;
    angle = (angle > 180) ? (angle - 360) : angle;
		return angle;
  }

  /**
   * Stops motors by calling tankDrive(0, 0).
   */
  public void stop() {
    tankDrive(0.0, 0.0);
  }

  /**
   * @return average between left and right encoders, in inches
   */
  public double getAverageDistance() {
		return (getRightEncoderInches() + getLeftEncoderInches()) / 2.0;
  }

  /**
   * Set up PID parameters for the drive train Talons
   * @param kP Proportional term
   * @param kI Integral term 
   * @param kD Differential term
   * @param kF Feed forward term (multiplied by setpoint)
   */
  public void setTalonPIDConstants(double kP, double kI, double kD, double kF) {
    leftMotor1.config_kP(0, kP);
    leftMotor1.config_kI(0, kI);
    leftMotor1.config_kD(0, kD);
    leftMotor1.config_kF(0, kF);

    rightMotor1.config_kP(0, kP);
    rightMotor1.config_kI(0, kI);
    rightMotor1.config_kD(0, kD);
    rightMotor1.config_kF(0, kF);

    leftMotor1.selectProfileSlot(0, 0);
    rightMotor1.selectProfileSlot(0, 0);
  }

  /**
   * Sets Talon to velocity closed-loop control mode with target velocity and feed-forward constant.
   * @param targetVel Target velocity, in inches per second
   * @param aFF Feed foward term to add to the contorl loop (-1 to +1)
   * @param reverseRight True = reverse velocity and FF term for right Talon
   */
  public void setTalonPIDVelocity(double targetVel, double aFF, boolean reverseRight) {
    int direction = (reverseRight) ? -1 : 1;
    leftMotor1.set(ControlMode.Velocity, 
      targetVel / kEncoderDistanceInchesPerPulse / 10.0, DemandType.ArbitraryFeedForward, aFF);
    rightMotor1.set(ControlMode.Velocity, 
      targetVel*direction  / kEncoderDistanceInchesPerPulse / 10.0, DemandType.ArbitraryFeedForward, aFF*direction);
    feedTheDog();
  }

  public double getLeftOutputPercent() {
    return leftMotor1.getMotorOutputPercent();
  }

  public double getTalonLeftClosedLoopError() {
    return leftMotor1.getClosedLoopError();
  }

  public double getTalonLeftClosedLoopTarget() {
    return leftMotor1.getClosedLoopTarget();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double degrees = getGyroRotation();
    double leftMeters = Units.inchesToMeters(getLeftEncoderInches());
    double rightMeters = Units.inchesToMeters(getRightEncoderInches());

    SmartDashboard.putNumber("Drive Right Encoder", getRightEncoderInches());
    SmartDashboard.putNumber("Drive Left Encoder", getLeftEncoderInches());
    SmartDashboard.putNumber("Drive Right Velocity", getRightEncoderVelocity());
    SmartDashboard.putNumber("Drive Left Velocity", getLeftEncoderVelocity());
    SmartDashboard.putNumber("Gyro Rotation", getGyroRotation());
    SmartDashboard.putNumber("Gyro Raw", getGyroRaw());

    odometry.update(Rotation2d.fromDegrees(-degrees), leftMeters, rightMeters);

    // track position from odometry (helpful for autos)
    var translation = odometry.getPoseMeters().getTranslation();
    SmartDashboard.putNumber("Odometry X",translation.getX());
    SmartDashboard.putNumber("Odometry Y",translation.getY());

    // TODO keep in code until values can be tuned for ACTUAL 2020 robot
     // save new current value for calculating angVel
     currAng = getGyroRaw();
     currTime = System.currentTimeMillis();
 
     // calculate angVel
     angularVelocity =  lfRunningAvg.calculate( (currAng - prevAng) / (currTime - prevTime) * 1000 );
 
     // convert angVel to degrees per sec & put on SmartDashboard
     SmartDashboard.putNumber("AngVel", angularVelocity);
 
     // save current angVel values as previous values for next calculation
     prevAng = currAng;
     prevTime = currTime; 
     
     if(log.getLogRotation() == log.DRIVE_CYCLE) {
      updateDriveLog(false);
    }
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * @return wheel speeds, in meters per second
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(Units.inchesToMeters(getLeftEncoderVelocity()), Units.inchesToMeters(getRightEncoderVelocity()));
  }

  /**
   * Start the timer for the autonomous period. Useful for comparing to generated trajectories.
   */
  public void startAutoTimer() {
    if (this.autoTimer == null) this.autoTimer = new Timer();
    this.autoTimer.reset();
    this.autoTimer.start();
  }

  /**
   * Set the voltage for the left and right motors (compensates for the current bus voltage)
   * @param leftVolts volts to output to the left motor
   * @param rightVolts volts to output to the right motor
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    if (autoTimer == null) {
      this.startAutoTimer();
    }

    leftMotor1.setVoltage(leftVolts);
    rightMotor1.setVoltage(rightVolts);
    feedTheDog();

    log.writeLogEcho(true, "TankDriveVolts", "Update", 
      "Time", autoTimer.get(), 
      "L Meters", Units.inchesToMeters(getLeftEncoderInches()),
      "R Meters", Units.inchesToMeters(getRightEncoderInches()), 
      "L Velocity", Units.inchesToMeters(getLeftEncoderVelocity()), 
      "R Velocity", Units.inchesToMeters(getRightEncoderVelocity()), 
      "L Volts", leftVolts, 
      "R Volts", rightVolts, 
      "Gyro", getGyroRotation());
  }

  /**
   * Writes information about the drive train to the filelog
   * @param logWhenDisabled true will log when disabled, false will discard the string
   */
  public void updateDriveLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, "Drive", "updates", 
      "L1 Volts", leftMotor1.getMotorOutputVoltage(), "L2 Volts", leftMotor2.getMotorOutputVoltage(),
      "L1 Amps", leftMotor1.getSupplyCurrent(), "L2 Amps", leftMotor2.getSupplyCurrent(),
      "L1 Temp",leftMotor1.getTemperature(), "L2 Temp",leftMotor2.getTemperature(),
      "R1 Volts", rightMotor1.getMotorOutputVoltage(), "R2 Volts", rightMotor2.getMotorOutputVoltage(),
      "R1 Amps", rightMotor1.getSupplyCurrent(), "R2 Amps", rightMotor2.getSupplyCurrent(), 
      "R1 Temp",rightMotor1.getTemperature(), "R2 Temp",rightMotor2.getTemperature(),
      "Left Inches", getLeftEncoderInches(), "L Vel", getLeftEncoderVelocity(),
      "Right Inches", getRightEncoderInches(), "R Vel", getRightEncoderVelocity(),
      "Gyro Angle", getGyroRotation(), "RawGyro", getGyroRaw(), "Time", System.currentTimeMillis()
      );
  }

  /**
   * Update TemperatureCheck utility with motors that are and are not overheating.
   */
  public void updateOverheatingMotors() {
    if (leftMotor1.getTemperature() >= temperatureCheck)
      tempCheck.recordOverheatingMotor("DriveLeft1");
    if (leftMotor2.getTemperature() >= temperatureCheck)
      tempCheck.recordOverheatingMotor("DriveLeft2");
    
    if (rightMotor1.getTemperature() >= temperatureCheck)
      tempCheck.recordOverheatingMotor("DriveRight1");
    if (rightMotor2.getTemperature() >= temperatureCheck)
      tempCheck.recordOverheatingMotor("DriveRight2");


    if (leftMotor1.getTemperature() < temperatureCheck)
      tempCheck.notOverheatingMotor("DriveLeft1");
    if (leftMotor1.getTemperature() < temperatureCheck)
      tempCheck.notOverheatingMotor("DriveLeft2");

    if (rightMotor1.getTemperature() < temperatureCheck)
      tempCheck.notOverheatingMotor("DriveRight1");
    if (rightMotor2.getTemperature() < temperatureCheck)
      tempCheck.notOverheatingMotor("DriveRight2");
  }
}
