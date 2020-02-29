/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TargetType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

import static frc.robot.Constants.DriveConstants.*;

public class DriveWithJoystickArcadeVelocity extends CommandBase {
  private DriveTrain driveTrain; // reference to driveTrain
  private FileLog log;
  private final Joystick leftJoystick, rightJoystick;

  /**
   * Drives the robot with correction to keep it driving straight.
   * @param target distance to travel, in meters
   * @param leftJoystick left joystick
   * @param rightJoystick right joystick
   * @param log
   */
  public DriveWithJoystickArcadeVelocity(DriveTrain driveTrain, Joystick leftJoystick, Joystick rightJoystick, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.log = log;
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    addRequirements(driveTrain);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setTalonPIDConstants(kPLinear, kILinear, kDLinear, 0);
    driveTrain.resetTalonPIDs();
    // driveTrain.setDriveModeCoast(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -leftJoystick.getY();
    double zRotation = rightJoystick.getX();

    // driveTrain.arcadeDrive(xSpeed, zRotation * 0.6);     // Note that arcadeDrive multiplies zRotation by an additional 0.7
    double rotateControl = 2;
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);

    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    zRotation = Math.copySign( Math.pow(Math.abs(zRotation), rotateControl), zRotation);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    if (squareInputs) {
      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
      zRotation = Math.copySign(zRotation * zRotation, zRotation);
    }

    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      } else {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      }
    }

    m_leftMotor.set(MathUtil.clamp(leftMotorOutput, -1.0, 1.0) * m_maxOutput);
    double maxOutput = m_maxOutput * m_rightSideInvertMultiplier;
    m_rightMotor.set(MathUtil.clamp(rightMotorOutput, -1.0, 1.0) * maxOutput);




    // For competition:  driving with feedforward and feedback
    driveTrain.setLeftTalonPIDVelocity(Units.metersToInches(targetVelL), aFFL);
    driveTrain.setRightTalonPIDVelocity(Units.metersToInches(targetVelR), aFFR, true);

    if(log.getLogRotation() == log.DRIVE_CYCLE) {
      log.writeLog(false, "DriveWithJoystickArcadeVelocity", "Joystick", "L Joystick", xSpeed, "R Joystick", zRotation);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setLeftMotorOutput(0);
    driveTrain.setRightMotorOutput(0);
    // driveTrain.setDriveModeCoast(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}