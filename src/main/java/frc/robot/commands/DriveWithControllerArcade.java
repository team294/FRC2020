package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/**
 * Command to drive using a controller in arcade mode
 */
public class DriveWithControllerArcade extends CommandBase {
  
  private final DriveTrain driveTrain;
  private double leftPercent, rightPercent;
  private XboxController xBoxController;

  /**
   * Command to drive using a controller in arcade mode
   */
  public DriveWithControllerArcade(DriveTrain driveTrain, XboxController xboxController) {
    this.driveTrain = driveTrain;
    this.xBoxController = xboxController;
    addRequirements(driveTrain);
  }

  @Override
  public void execute() {
    leftPercent = xBoxController.getX(Hand.kRight)*.5;
    rightPercent = -xBoxController.getY(Hand.kLeft)*.35;

    driveTrain.arcadeDrive(leftPercent, rightPercent);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
