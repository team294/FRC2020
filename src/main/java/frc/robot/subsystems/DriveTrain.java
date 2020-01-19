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
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;


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

  
  public DriveTrain() {
    // Configure navX
    AHRS gyro = null;
		try {
      gyro = new AHRS(I2C.Port.kMXP);
      gyro.zeroYaw();
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
    ahrs = gyro;
    
    // configure motors
    leftMotor1 = new WPI_TalonFX(DriveConstants.leftDriveMotorOne);
    leftMotor2 = new WPI_TalonFX(DriveConstants.leftDriveMotorTwo);
    rightMotor1 = new WPI_TalonFX(DriveConstants.rightDriveMotorOne);
    rightMotor2 = new WPI_TalonFX(DriveConstants.rightDriveMotorTwo);

    leftMotor1.configFactoryDefault();
    leftMotor2.configFactoryDefault();
    rightMotor1.configFactoryDefault();
    rightMotor2.configFactoryDefault();

    leftMotor2.set(ControlMode.Follower, DriveConstants.leftDriveMotorOne);
    rightMotor2.set(ControlMode.Follower, DriveConstants.rightDriveMotorOne);

    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);

    leftMotor1.setInverted(true); //TODO invert motors based on actual robot
    leftMotor2.setInverted(true);
    rightMotor1.setInverted(true);
    rightMotor2.setInverted(true);

    setDriveModeCoast(false);

    leftMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rightMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    
    leftMotor1.setSensorPhase(false); //TODO invert encoders based on actual robot
    rightMotor1.setSensorPhase(false);

    leftMotor1.configVoltageCompSaturation(12.0);
    leftMotor2.configVoltageCompSaturation(12.0);
    rightMotor1.configVoltageCompSaturation(12.0);
    rightMotor2.configVoltageCompSaturation(12.0);

    // create the drive train AFTER configuring the motors
    driveTrain = new DifferentialDrive(leftMotor1, rightMotor1);
    driveTrain.setDeadband(0.05);
    
    zeroLeftEncoder();
    zeroRightEncoder();

    zeroGyroRotation();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getGyroRotation()));
  }


  /**
   * Tank drive method for differential drive platform.
   * The calculated values will be squared to decrease sensitivity at low speeds.
   *
   * @param leftPercent  The robot's left side percent along the X axis [-1.0..1.0]. Forward is
   *                   positive.
   * @param rightPercent The robot's right side percent along the X axis [-1.0..1.0]. Forward is
   *                   positive.
   */
  public void tankDrive(double leftPercent, double rightPercent) {
    driveTrain.tankDrive(leftPercent, rightPercent, true);
  }

  /**
   * Tank drive method for differential drive platform.
   *
   * @param leftPercent  The robot's left side percent along the X axis [-1.0..1.0]. Forward is
   *                   positive.
   * @param rightPercent The robot's right side percent along the X axis [-1.0..1.0]. Forward is
   *                   positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  public void tankDrive(double leftPercent, double rightPercent, boolean squareInputs) {
    driveTrain.tankDrive(leftPercent, rightPercent, squareInputs);
  }

  public void setLeftMotorOutput(double percent) {
    leftMotor1.set(ControlMode.PercentOutput, percent);
  }

  public void setRightMotorOutput(double percent) {
    rightMotor1.set(ControlMode.PercentOutput, percent);
  }

  public void arcadeDrive(double speedPct, double rotation) {
    driveTrain.arcadeDrive(speedPct, rotation, false);
  }

  public void setVoltageCompensation(boolean turnOn) {
    leftMotor1.enableVoltageCompensation(turnOn);
    leftMotor2.enableVoltageCompensation(turnOn);
    rightMotor1.enableVoltageCompensation(turnOn);
    rightMotor2.enableVoltageCompensation(turnOn);
  }
  public double getLeftEncoderRaw() {
    return leftMotor1.getSelectedSensorPosition(0);
  }

  public double getRightEncoderRaw() {
    return rightMotor1.getSelectedSensorPosition(0);
  }

  public double getLeftEncoderVelocityRaw() {
    return leftMotor1.getSelectedSensorVelocity(0);
  }

  public double getRightEncoderVelocityRaw() {
    return rightMotor1.getSelectedSensorVelocity(0);
  }

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
	 * Zeros the left encoder position in software
	 */
  public void zeroLeftEncoder() {
    leftEncoderZero = getLeftEncoderRaw();
  }

  /**
	 * Zeros the right encoder position in software
	 */
  public void zeroRightEncoder() {
    rightEncoderZero = getRightEncoderRaw();
  }

  /**
	 * Get the position of the left encoder, in encoder ticks since last zeroLeftEncoder()
	 * 
	 * @return encoder position, in ticks
	 */
  public double getLeftEncoderTicks() {
    return getLeftEncoderRaw() - leftEncoderZero;
  }

  /**
	 * Get the position of the right encoder, in encoder ticks since last zeroRightEncoder()
	 * 
	 * @return encoder position, in ticks
	 */
  public double getRightEncoderTicks() {
    return -(getRightEncoderRaw() - rightEncoderZero);
  }

  public static double encoderTicksToInches(double ticks) {
    return ticks / DriveConstants.ticksPerInch;
  }

  public double getLeftEncoderInches() {
    return encoderTicksToInches(getLeftEncoderTicks());
  }

  public double getRightEncoderInches() {
    return encoderTicksToInches(getRightEncoderTicks());
  }

  public static double inchesToMeters(double number){
    return number * 0.0254;
  }

  /**
   * 
   * @return left encoder velocity in inches / sec
   */
  public double getLeftEncoderVelocity() {
    return encoderTicksToInches(getLeftEncoderVelocityRaw()) * 10;
  }

    /**
   * @return right encoder velocity in inches / sec
   */
  public double getRightEncoderVelocity() {
    return encoderTicksToInches(getRightEncoderVelocityRaw()) * 10;
  }

  public double inchesToEncoderTicks(double inches) {
    return inches * DriveConstants.ticksPerInch;
  }

  /**
   * Gets the raw value of the gyro in degrees
   * @return
   */
  public double getGyroRaw() {
    return ahrs.getAngle();
  }

  /**
	 * Zeros the gyro position in software
	 */
	public void zeroGyroRotation() {
		// set yawZero to gryo angle
    yawZero = getGyroRaw();
  }
  
  /**
	 * Resets the gyro position in software to a specified angle
	 * 
	 * @param currentHeading Gyro heading to reset to, in degrees
	 */
	public void setGyroRotation(double currentHeading) {
		// set yawZero to gryo angle, offset to currentHeading
		yawZero = getGyroRaw() - currentHeading;
		// System.err.println("PLZ Never Zero the Gyro Rotation it is not good");
  }

  /**
	 * Gets the rotation of the gyro
	 * 
	 * @return Current angle from -180 to 180 degrees
	 */
	public double getGyroRotation() {
		double angle = getGyroRaw() - yawZero;
		// Angle will be in terms of raw gyro units (-inf,inf), so you need to convert
		// to (-180, 180]
		angle = normalizeAngle(angle);
		return angle;
  }

  /**
	 * Converts the input angle to a number between -179.999 and +180.0
	 * 
	 * @return Normalized angle
	 */
	public static double normalizeAngle(double angle) {
		angle = angle % 360;
		angle = (angle <= -180) ? (angle + 360) : angle;
    angle = (angle > 180) ? (angle - 360) : angle;
		return angle;
  }

  /**
   * Stops the motors by calling tankDrive(0, 0)
   */
  public void stop() {
    tankDrive(0.0, 0.0);
  }

  public double getAverageDistance() {
		return (getRightEncoderInches() + getLeftEncoderInches()) / 2.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double degrees = getGyroRotation();
    double leftMeters = inchesToMeters(getLeftEncoderInches());
    double rightMeters = inchesToMeters(getRightEncoderInches());

    SmartDashboard.putNumber("Right Encoder", getRightEncoderInches());
    SmartDashboard.putNumber("Left Encoder", getLeftEncoderInches());
    SmartDashboard.putNumber("Gyro Rotation", getGyroRotation());

    odometry.update(Rotation2d.fromDegrees(degrees), leftMeters, rightMeters);
    
  }

  public Pose2d getPose() {
    System.out.println("Position" + odometry.getPoseMeters());
    System.out.println("Gyro Rotation " + getGyroRotation() + ", Right Meters " + 
      inchesToMeters(getRightEncoderInches()) + ", Left Meters " + 
      inchesToMeters(getLeftEncoderInches()));
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(Units.inchesToMeters(getLeftEncoderVelocity()), Units.inchesToMeters(getRightEncoderVelocity()));
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotor1.setVoltage(leftVolts);
    rightMotor1.setVoltage(-rightVolts);
    System.out.println("left Volts " + leftVolts);
    System.out.println("right Volts " + -rightVolts);
  }
}