/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class HopperReverse extends CommandBase {
  private Hopper hopper;
  private Shooter shooter;
  private Timer shooterTimer, hopperTimer;

  public HopperReverse(Hopper hopper, Shooter shooter) {
    this.hopper = hopper;
    this.shooter = shooter;
    shooterTimer = new Timer();
    hopperTimer = new Timer();
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hopperTimer.reset();
    shooterTimer.reset();
    shooterTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterTimer.hasPeriodPassed(0.75) && shooter.getVoltage() < Constants.ShooterConstants.voltageCheck && hopperTimer.get() == 0) {
      hopperTimer.start();
      hopper.hopperSetPercentOutput(-0.8);
    } else if (hopperTimer.hasPeriodPassed(1)) {
      hopperTimer.stop();
      shooterTimer.stop();
      hopperTimer.reset();
      shooterTimer.reset();
      shooterTimer.start();
      hopper.hopperSetPercentOutput(0.8);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopperTimer.stop();
    shooterTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
