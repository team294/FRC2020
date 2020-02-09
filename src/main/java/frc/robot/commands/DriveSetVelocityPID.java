/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class DriveSetVelocityPID extends CommandBase {
  /**
   * Creates a new DriveSetVelocityPID.
   */
  private DriveTrain driveTrain;
  private FileLog log;
  private double targetVel;
  private double kP, kI, kD, kF;

  public DriveSetVelocityPID(double targetVel, DriveTrain driveTrain, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetVel = targetVel;
    this.driveTrain = driveTrain;
    this.log = log;
    addRequirements(driveTrain);
    kP = 0.1;
    kI = 0.0;
    kD = 0.0;
    kF = 0.0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setTalonPIDConstants(kP, kI, kD, kF);
    driveTrain.setTalonPIDVelocity(targetVel, 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
