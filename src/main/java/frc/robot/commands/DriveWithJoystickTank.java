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

public class DriveWithJoystickTank extends CommandBase {
  
  private final DriveTrain driveTrain;
  private final Joystick leftJoystick;
  private final Joystick rightJoystick;
  
  private double leftPercent, rightPercent;

  /**
   * Creates a new DriveWithJoystickArcade.
   */
  public DriveWithJoystickTank(DriveTrain driveTrain, Joystick leftJoystick, Joystick rightJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
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
    leftPercent = leftJoystick.getY();
    rightPercent = rightJoystick.getX();

    if(Math.abs(leftPercent) < 0.05){
      leftPercent = 0;
    }
    if(Math.abs(rightPercent) < 0.05){
      rightPercent = 0;
    }

    driveTrain.setPercentOutputTank(leftPercent, rightPercent);
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
