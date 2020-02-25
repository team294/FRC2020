/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.Timer;

public class LEDRainbow extends CommandBase {
  private LED led;
  private int strip; // which strip to set rainbow
  private double speed; // how often to shift rainbow, in seconds
  private Timer timer;
  private int patternNum = 0;
 
  /**
   * Set LED strip to a shifting rainbow, with parameter seconds between each shift.
   * This command never ends.
   * @param strip strip number (0 or 1)
   * @param speed how often to shift rainbow, in seconds
   * @param led led strip (subsystem)
   */
  public LEDRainbow(int strip, double speed, LED led) {
    this.led = led;
    this.strip = strip;
    this.speed = speed;
    this.timer = new Timer();
    addRequirements(led);
  }

  /**
   * Set LED strip to a shifting rainbow, with 0.5 seconds between each shift.
   * This command never ends.
   * @param strip strip number (0 or 1)
   * @param led led strip (subsystem)
   */
  public LEDRainbow(int strip, LED led) {
    this.led = led;
    this.strip = strip;
    this.speed = 0.5;
    this.timer = new Timer();
    addRequirements(led);
  }

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
    timer.reset();
    timer.start();
  }
      
  // Called every time the scheduler runs while the command is scheduled.
  @Override
	public void execute() {  
    if(patternNum > LED.rainbowLibrary.length - 1) patternNum = 0;
    
    led.setPattern(LED.rainbowLibrary[patternNum], 0.5, strip);
    
    if(timer.hasPeriodPassed(speed)) {
      patternNum++;
      timer.reset();
      timer.start();
    }
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
