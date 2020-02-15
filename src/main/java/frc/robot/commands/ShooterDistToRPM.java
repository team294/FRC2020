/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterDistToRPM extends CommandBase {
  /**
   * Uses method in shooter to calculate RPM when at a certain distance away from the target
   * Will not end if inputting sample dists from SmartDashboard (for continuous calculations)
   */
  private Shooter shooter;
  private double dist;
  private boolean fromSmartDashboard;

  public ShooterDistToRPM(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.dist = 0;
    this.fromSmartDashboard = true;
    addRequirements(shooter);

    if(SmartDashboard.getNumber("Shooter Distance", -9999) == -9999)
    SmartDashboard.putNumber("Shooter Distance", 5);
  }

  /**
   * @param dist distance from the target in feet
   * @param shooter reference to shooter subsystem
   */
  public ShooterDistToRPM(double dist, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.dist = dist;
    this.fromSmartDashboard = false;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(fromSmartDashboard) dist = SmartDashboard.getNumber("Shooter Distance", 5);
    double rpm = shooter.distanceFromTargetToRPM(dist);
    SmartDashboard.putNumber("Shooter RPM from Dist", rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dist = SmartDashboard.getNumber("Shooter Distance", 5);
    double rpm = shooter.distanceFromTargetToRPM(dist);
    SmartDashboard.putNumber("Shooter RPM from Dist", rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !fromSmartDashboard;
  }
}
