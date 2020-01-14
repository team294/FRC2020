/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import javax.management.loading.PrivateMLet;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */

 
  private final WPI_TalonFX leftMotor1 = new WPI_TalonFX(Constants.leftDriveMotorOne);
  private final WPI_TalonFX leftMotor2 = new WPI_TalonFX(Constants.leftDriveMotorTwo);
  private final WPI_TalonFX rightMotor1 = new WPI_TalonFX(Constants.rightDriveMotorOne);
  private final WPI_TalonFX rightMotor2 = new WPI_TalonFX(Constants.rightDriveMotorTwo);

  private double leftEncoderZero = 0, rightEncoderZero = 0;

  private final DifferentialDrive driveTrain = new DifferentialDrive(leftMotor1, rightMotor1);

  AHRS ahrs;
  private double yawZero = 0;

  
  public DriveTrain() {
    // Configure navX
		try {
			/* Communicate w/navX MXP via the MXP SPI Bus.
			 * Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB
			 * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
			 * details.
			 */

			ahrs = new AHRS(I2C.Port.kMXP);

		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
    ahrs.zeroYaw();

    leftMotor2.set(ControlMode.Follower, Constants.leftDriveMotorOne);
    rightMotor2.set(ControlMode.Follower, Constants.rightDriveMotorOne);

    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);

    leftMotor1.setInverted(true); //TODO invert motors based on actual robot
    leftMotor2.setInverted(true);
    rightMotor1.setInverted(true);
    rightMotor2.setInverted(true);

    leftMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.SensorDifference, 0, 0);
    rightMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.SensorDifference, 0, 0);
    
    leftMotor1.setSensorPhase(true); //TODO invert encoders based on actual robot
    rightMotor1.setSensorPhase(true);

    driveTrain.setDeadband(0.12); //TODO set deadband based on robot is currently set to last year's value

    leftMotor1.configVoltageCompSaturation(12.0);
    leftMotor2.configVoltageCompSaturation(12.0);
    rightMotor1.configVoltageCompSaturation(12.0);
    rightMotor2.configVoltageCompSaturation(12.0);
    
  }
  /**
   * Set percent output for tank drive.
   * @param leftOut percent output, left side
   * @param rightOut percent output, right side
   */
  public void setPercentOutputTank(double leftOut, double rightOut) {
    driveTrain.tankDrive(leftOut, rightOut);
  }

  /**
   * 
   * @param powerLeft percent output, left side
   * @param powerRight percent output, right side
   * @param squaredInputs true if want to have exponential control of speed
   */
  public void setPercentOutputTank(double powerLeft, double powerRight, boolean squaredInputs) {
    driveTrain.tankDrive(powerLeft, powerRight, squaredInputs);
  }

  public void setLeftMotorOutput(double percent) {
    leftMotor1.set(ControlMode.PercentOutput, percent);
  }

  public void setRightMotorOutput(double percent) {
    rightMotor1.set(ControlMode.PercentOutput, percent);
  }

  public void setPercentOutputArade(double speedPct, double rotation) {
    driveTrain.arcadeDrive(speedPct, rotation, false);
  }

  public void setVoltageCompensation(boolean turnOn) {
    leftMotor1.enableVoltageCompensation(turnOn);
    leftMotor2.enableVoltageCompensation(turnOn);
    rightMotor1.enableVoltageCompensation(turnOn);
    rightMotor2.enableVoltageCompensation(turnOn);
  }
  public double getLeftEncoderRaw() {
    return leftMotor2.getSelectedSensorPosition(0);
  }

  public double getRightEncoderRaw() {
    return rightMotor1.getSelectedSensorPosition(0);
  }

  public double getLeftEncoderVelocityRaw() {
    return leftMotor2.getSelectedSensorVelocity(0);
  }

  public double getRightEncoderVelocityRaw() {
    return rightMotor2.getSelectedSensorVelocity(0);
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

  public double encoderTicksToInches(double ticks) {
    return ticks * Constants.wheelCircumference / Constants.encoderTicksPerRevolution;
  }

  public double getLeftEncoderInches() {
    return encoderTicksToInches(getLeftEncoderTicks());
  }

  public double getRightEncoderInches() {
    return encoderTicksToInches(getRightEncoderTicks());
  }

  /**
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
    return (inches / Constants.wheelCircumference) * Constants.encoderTicksPerRevolution;
  }

  /**
   * Gets the raw value of the gyro
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
	public double normalizeAngle(double angle) {
		angle = angle % 360;
		angle = (angle <= -180) ? (angle + 360) : angle;
    angle = (angle > 180) ? (angle - 360) : angle;
		return angle;
  }

  /**
   * Stops the motors by calling tankDrive(0, 0)
   */
  public void stop() {
    setPercentOutputTank(0.0, 0.0);
  }

  public double getAverageDistance() {
		return (getRightEncoderInches() + getLeftEncoderInches()) / 2.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
