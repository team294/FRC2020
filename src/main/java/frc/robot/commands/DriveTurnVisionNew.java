/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.utilities.FileLog;

public class DriveTurnVisionNew extends CommandBase {
  /**
   * Creates a new DriveTurnVisionNew.
   */
  private final DriveTrain driveTrain;
  private final FileLog log;
  private final Timer timer;
  private final LimeLight limeLight;
  private double currAng;
  private double targetAbs;
  private double targetRel;
  private double direction;
  private boolean useVision;
  private int accuracyCounter;

  public DriveTurnVisionNew(boolean useVision, DriveTrain driveTrain, LimeLight limeLight, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.limeLight = limeLight;
    this.log = log;
    this.useVision = useVision;
    this.timer = new Timer();
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "DriveTurnVisionNew", "Init");
    if(useVision) {
      targetRel = driveTrain.normalizeAngle(limeLight.getXOffset());
    }

    if(targetRel < 0) {
      direction = 1;
    } else {
      direction = -1;
    }

    currAng = driveTrain.getGyroRotation();
    targetAbs = currAng + targetRel;
    
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(targetAbs - currAng) < 1) {
      log.writeLog(false, "DriveTurnVisionNew", "Motor %", 0.0);
      driveTrain.arcadeDrive(0, 0);
    } else if (!timer.hasElapsed(0.05)) {
      driveTrain.arcadeDrive(0, (0.15 * direction));
      log.writeLog(false, "DriveTurnVisionNew", "Motor %", 0.15);
    } else {
      driveTrain.arcadeDrive(0, (0.1 * direction));
      log.writeLog(false, "DriveTurnVisionNew", "Motor %", 0.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(targetAbs-currAng) < 1) {
      log.writeLog(false, "DriveTurnVisionNew", "WithinToleranceCount", accuracyCounter);
      accuracyCounter++;
    } else {
      accuracyCounter = 0;
    }

    if(accuracyCounter >= 5) {
      return true;
    }
    return false;
  }
}
