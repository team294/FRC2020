/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Timer;


/**
 * Command to send a solid color to the LED strip.
 */
public class LEDRainbow extends CommandBase {
  private LED led;
  private int ledStrip;
  private double intensity = 0.5;
  private Color[][] rainbowLibrary = LED.rainbowLibrary;
  private Timer timer = new Timer();;
  private double duration = 0.05;
 

  /**
   * Only use this to turn off LEDs.
   * @param color color as a string (first letter capital)
   * @param led LED subsystem to use
   **/
	public LEDRainbow(LED led, int ledStrip) {
    this.led = led;
    this.ledStrip = ledStrip;
    addRequirements(led);
  
  }
  
  /**
   * @param color color as a string (first letter capital)
   * @param intensity LED intensity (0 to 1)
   * @param led LED subsystem to use
   **/
	public LEDRainbow(int ledStrip, double intensity, LED led) {
    this.led = led;
    this.ledStrip = ledStrip;
    this.intensity = intensity;
	  addRequirements(led);
  }

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
    timer.reset();
    timer.start();
  }
    
  private int patternNum = 0;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
	public void execute() {  
    if(patternNum > rainbowLibrary.length - 1){
      patternNum = 0;
    }
    
    led.setPattern(rainbowLibrary[patternNum], intensity, ledStrip);
    
    if(timer.hasPeriodPassed(duration)){
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
