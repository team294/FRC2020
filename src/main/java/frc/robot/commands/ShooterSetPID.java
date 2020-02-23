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
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LimeLight;

public class ShooterSetPID extends CommandBase {
  private final Shooter shooter;
  private LimeLight limeLight;
  private LED led;
  private double rpm;
  private boolean rpmFromShuffleboard, rpmFromDistance;
  private Timer ledTimer;
  private String ledColor = "Blue";
  
  /**
   * Set shooter PID using parameter RPM.
   * This command ends when shooter RPM is within tolerance.
   * @param rpm setpoint in RPM
   * @param shooter shooter subsystem
   */
  public ShooterSetPID(double rpm, Shooter shooter, LED led) {
    this.shooter = shooter;
    this.led = led;
    this.rpm = rpm;
    this.rpmFromShuffleboard = false;
    this.rpmFromDistance = false;
    this.ledTimer = new Timer();
    addRequirements(shooter);
  }

  /**
   * Set shooter PID using either RPM from distance to target or RPM from shuffleboard.
   * This command ends when shooter RPM is within tolerance.
   * @param rpmFromDistance true = RPM using distance from target, false = RPM using shuffleboard
   * @param shooter shooter subsystem
   */
  public ShooterSetPID(boolean rpmFromDistance, Shooter shooter, LimeLight limeLight, LED led) {
    this.shooter = shooter;
    this.limeLight = limeLight;
    this.led = led;
    this.rpm = 0;
    this.rpmFromShuffleboard = !rpmFromDistance;
    this.rpmFromDistance = rpmFromDistance;
    this.ledTimer = new Timer();
    addRequirements(shooter);

    if(SmartDashboard.getNumber("Shooter Manual SetPoint RPM", -9999) == -9999)
      SmartDashboard.putNumber("Shooter Manual SetPoint RPM", 2800);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(rpmFromShuffleboard && !rpmFromDistance) rpm = SmartDashboard.getNumber("Shooter Manual SetPoint RPM", 2800);
    else if(rpmFromDistance) rpm = shooter.distanceFromTargetToRPM(limeLight.getDistanceNew());
    shooter.setShooterPID(rpm);
    ledTimer.reset();
    ledTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("LED Color", ledColor);

    if(ledTimer.hasPeriodPassed(0.1)) {
      led.setStrip(ledColor, 0.5, 1);
      ledTimer.reset();
      ledTimer.start();
      if(ledColor.equals("Blue")) ledColor = "Black";
      else ledColor = "Blue";
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) shooter.setShooterVoltage(0);
    ledTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(shooter.getMeasuredRPM() - rpm) < 200) {
      led.setStrip("Blue", 0.5, 1);
      return true;
    } else return false;
  }
}
