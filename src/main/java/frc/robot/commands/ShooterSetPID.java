/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/**
 * Command to set the shooter PID.
 */
public class ShooterSetPID extends CommandBase {
  private final Shooter shooter;
  private double rpm;
  private final boolean getRpmFromShuffleboard;
  private final boolean rpmFromDistance;
  private Timer timer;
  
  /**
   * @param rpm setpoint in RPM
   * @param shooter shooter subsystem to use
   */
  public ShooterSetPID(double rpm, Shooter shooter) {
    this.shooter = shooter;
    this.rpm = rpm;
    this.getRpmFromShuffleboard = false;
    this.rpmFromDistance = false;
    timer = new Timer();
    addRequirements(shooter);
  }

  /**
   * Turn on the shooter PID using RPM from Shuffleboard.
   * @param rpmFromDistance true = rpm is set with distance from target, false = rpm is set with manual dashboard input
   * @param shooter Shooter subsystem to use
   */
  public ShooterSetPID(boolean rpmFromDistance, Shooter shooter) {
    this.shooter = shooter;
    this.rpm = 0;
    getRpmFromShuffleboard = true;
    this.rpmFromDistance = rpmFromDistance;
    timer = new Timer();
    addRequirements(shooter);

    if(SmartDashboard.getNumber("Shooter Manual SetPoint RPM", -9999) == -9999)
      SmartDashboard.putNumber("Shooter Manual SetPoint RPM", 2800);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(getRpmFromShuffleboard && !rpmFromDistance) rpm = SmartDashboard.getNumber("Shooter Manual SetPoint RPM", 2800);
    else if(getRpmFromShuffleboard && rpmFromDistance) rpm = shooter.distanceFromTargetToRPM(SmartDashboard.getNumber("Shooter Distance", 5));
    shooter.setShooterPID(rpm);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) shooter.setShooterVoltage(0);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if(timer.hasPeriodPassed(0.1) && Math.abs(shooter.getShooterPIDError()) < 200) return true;
    if (Math.abs(shooter.getMeasuredRPM() - rpm) < 200) return true;
    else return false;
  }
}
