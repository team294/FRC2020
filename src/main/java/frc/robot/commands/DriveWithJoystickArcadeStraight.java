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
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

import static frc.robot.Constants.DriveConstants.*;

public class DriveWithJoystickArcadeStraight extends CommandBase {
  private final DriveTrain driveTrain; // reference to driveTrain
  private final FileLog log;
  private final Joystick leftJoystick, rightJoystick;
  private boolean driveStraight;
  // private double priorLeftEncoder, priorRightEncoder;
  private double priorAngle;

  /**
   * Drives the robot with correction to keep it driving straight.
   * @param target distance to travel, in meters
   * @param leftJoystick left joystick
   * @param rightJoystick right joystick
   * @param log
   */
  public DriveWithJoystickArcadeStraight(DriveTrain driveTrain, Joystick leftJoystick, Joystick rightJoystick, FileLog log) {
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
    // priorLeftEncoder = driveTrain.getLeftEncoderInches();
    // priorRightEncoder = driveTrain.getRightEncoderInches();
    priorAngle = driveTrain.getGyroRotation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -leftJoystick.getY();
    double zRotation = rightJoystick.getX();
    double rRaw = zRotation;

    // Are we trying to drive straight?
    driveStraight = (Math.abs(zRotation) < 0.02) && (Math.abs(xSpeed) > 0.02);

    // Algorithm copied from DifferentialDrive.arcadeDrive, with modification for zRotation sensitivity
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    zRotation = Math.copySign( Math.pow(Math.abs(zRotation), joystickTurnSensitivity), zRotation);

    // double distLeft = driveTrain.getLeftEncoderInches() - priorLeftEncoder;
    // double distRight = driveTrain.getRightEncoderInches() - priorRightEncoder;
    double angleRelative = driveTrain.normalizeAngle(driveTrain.getGyroRotation() - priorAngle);

    if (driveStraight) {
      // zRotation = MathUtil.clamp((distRight - distLeft) * 0.5, -1.0, 1.0) * Math.abs(xSpeed);
      zRotation = MathUtil.clamp(angleRelative * kAngLinear * 0.2, -Math.abs(xSpeed), Math.abs(xSpeed));
    } else {
      zRotation *= 0.42;    // minimize max rotation speed from joystick

      // Reset the saved encoder values for the next time that we start driving straight
      // priorLeftEncoder = driveTrain.getLeftEncoderInches();
      // priorRightEncoder = driveTrain.getRightEncoderInches();
      priorAngle = driveTrain.getGyroRotation();
    }

    driveTrain.arcadeDrive(xSpeed, zRotation);

    // double leftMotorOutput;
    // double rightMotorOutput;

    // double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    // if (xSpeed >= 0.0) {
    //   // First quadrant, else second quadrant
    //   if (zRotation >= 0.0) {
    //     leftMotorOutput = maxInput;
    //     rightMotorOutput = xSpeed - zRotation;
    //   } else {
    //     leftMotorOutput = xSpeed + zRotation;
    //     rightMotorOutput = maxInput;
    //   }
    // } else {
    //   // Third quadrant, else fourth quadrant
    //   if (zRotation >= 0.0) {
    //     leftMotorOutput = xSpeed + zRotation;
    //     rightMotorOutput = maxInput;
    //   } else {
    //     leftMotorOutput = maxInput;
    //     rightMotorOutput = xSpeed - zRotation;
    //   }
    // }

    // leftMotorOutput = MathUtil.clamp(leftMotorOutput, -1.0, 1.0);
    // rightMotorOutput = MathUtil.clamp(rightMotorOutput, -1.0, 1.0);


    if(log.getLogRotation() == log.DRIVE_CYCLE || true) {
      // log.writeLog(false, "DriveWithJoystickArcadeVelocity", "Joystick", "L Joystick", xSpeed, "R Joystick", zRotation,
      //   "velLT", targetVelL, "velRT", targetVelR, 
      //   "velLA", Units.inchesToMeters(driveTrain.getLeftEncoderVelocity()), 
      //   "velRA", Units.inchesToMeters(driveTrain.getRightEncoderVelocity()), 
      //   "aFFL", aFFL, "aFFR", aFFR,
      //   "pctOutLA", driveTrain.getLeftOutputPercent(), "VoutNormA", driveTrain.getLeftOutputVoltage()/compensationVoltage, "VbusLA", driveTrain.getLeftBusVoltage(),
      //   "velRawLA", driveTrain.getLeftEncoderVelocityRaw(), "errRawLA", driveTrain.getTalonLeftClosedLoopError(), 
      //   "targetRawL", driveTrain.getTalonLeftClosedLoopTarget()
      // );
      log.writeLog(false, "DriveWithJoystickArcadeStraight", "Joystick", "L Joystick", xSpeed, "R Joystick", rRaw, "zRotation", zRotation,
        "velLA", driveTrain.getLeftEncoderVelocity(), 
        "velRA", driveTrain.getRightEncoderVelocity(),
        // "distLeft", distLeft, "distRight", distRight, "deltaDist", distRight-distLeft,
        "gyro", driveTrain.getGyroRotation(), "deltaAngle", angleRelative
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setLeftMotorOutput(0);
    driveTrain.setRightMotorOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}