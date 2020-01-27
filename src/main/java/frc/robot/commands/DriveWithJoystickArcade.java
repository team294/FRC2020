/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.*;

public class DriveWithJoystickArcade extends CommandBase {
  
  private final DriveTrain driveTrain;
  private final Joystick leftJoystick;
  private final Joystick rightJoystick;
  private final FileLog log;
  
  private double leftPercent, rightPercent;

  /**
   * Creates a new DriveWithJoystickArcade.
   * 
   * 
   */
  public <Filelog> DriveWithJoystickArcade(DriveTrain driveTrain, FileLog log, Joystick leftJoystick,
      Joystick rightJoystick) {
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

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftPercent = -leftJoystick.getY();
    rightPercent = rightJoystick.getX();

    log.writeLog(false, "DriveWithJoysticks", "Joystick", "L Joystick", leftPercent, "R Joystick", rightPercent);
    if(Math.abs(leftPercent) < 0.05){
      leftPercent = 0;
    }
    if(Math.abs(rightPercent) < 0.05){
      rightPercent = 0;
    }

    driveTrain.arcadeDrive(leftPercent, rightPercent * 0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
