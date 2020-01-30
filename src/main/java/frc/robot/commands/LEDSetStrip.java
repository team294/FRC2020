/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;

/**
 * Command to send a solid color to the LED strip.
 */
public class LEDSetStrip extends CommandBase {
  private LED led;
  private String color;
  private double intensity = -1;

  /**
   * Only use this to turn off LEDs.
   * @param color color as a string (first letter capital)
   * @param led LED subsystem to use
   **/
	public LEDSetStrip(String color, LED led) {
    this.led = led;
    this.color = color;
	  addRequirements(led);
  }
  
  /**
   * @param color color as a string (first letter capital)
   * @param intensity LED intensity (0 to 1)
   * @param led LED subsystem to use
   **/
	public LEDSetStrip(String color, double intensity, LED led) {
    this.led = led;
    this.color = color;
	  addRequirements(led);
  }

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
    if (intensity >= 0) led.setStrip(color, intensity);
		else led.setStrip(color);
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
