/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/**
 * Command to send a pattern to the LED strip.
 */
public class LEDSetPattern extends CommandBase {
  private LED led;
  private int rowNumber;
  private double intensity;
 // private LED led2; 
  
  /**
   * @param led LED subsystem to use
   * @param rowNumber row in the patternLibrary
   * @param intensity LED intensity (0 to 1)
   */
  public LEDSetPattern(LED led, int rowNumber, double intensity) {
    this.intensity = intensity;
    this.led = led;
    //this.led = led2; 
    this.rowNumber = rowNumber;
    addRequirements(led);
  }    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.setPattern(LED.patternLibrary[rowNumber], intensity);
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
    return true;
  }
}
