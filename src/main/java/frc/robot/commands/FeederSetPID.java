/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;

/**
 * Command to set the feeder PID.
 */
public class FeederSetPID extends CommandBase {
  private Feeder feeder;
  private double rpm;
  private boolean getRPMFromShuffleboard;

  /**
   * @param rpm setpoint
   * @param feeder feeder subsystem to use
   */
  public FeederSetPID(int rpm, Feeder feeder) {
    this.feeder = feeder;
    this.rpm = rpm;
    this.getRPMFromShuffleboard = false;
    addRequirements(feeder);
  }

  public FeederSetPID(Feeder feeder) {
    this.feeder = feeder;
    this.rpm = 0;
    getRPMFromShuffleboard = true;
    addRequirements(feeder);

    if (SmartDashboard.getNumber("Feeder Manual SetPoint RPM", -9999) == -9999)
      SmartDashboard.putNumber("Feeder Manual SetPoint RPM", 2000);
  }

  public FeederSetPID(boolean fromShuffleboard, Feeder feeder) {
    this.feeder = feeder;
    this.getRPMFromShuffleboard = false;
    this.rpm = Constants.FeederConstants.feederDefaultRPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (getRPMFromShuffleboard) rpm = SmartDashboard.getNumber("Feeder Manual SetPoint RPM", 2000);
    feeder.setFeederPID(rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(feeder.getFeederPIDError()) < 200) return true;
    else return false;
  }
}
