/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Color2;
import frc.robot.utilities.ColorSensor;

import static frc.robot.utilities.Color2.*;

public class LED extends SubsystemBase {
  private AddressableLED led; // strip
  private AddressableLEDBuffer ledBuffer; // data passed to strip
  private final int length = 32; // length of strip in pixels
  private String prevColor, currColor; // colors on the control panel
  private final ColorSensor colorSensor; // Reference to the color sensor

  public static final Color2[][] patternLibrary = {
    {kGreen, kFirstBlue, kWhite, kFirstBlue, kWhite, kFirstBlue, kWhite, kFirstBlue},
    {kGreen, kIndianRed, kWhite, kIndianRed, kWhite, kIndianRed, kWhite, kIndianRed}
  };
  
  /**
   * Controls LED strips on the robot.
   */
  public LED (ColorSensor colorSensor) {
    led = new AddressableLED(0); // must be a PWM port
                                 // currently port 0 for testing on miniBot
    ledBuffer = new AddressableLEDBuffer(length);
    led.setLength(length);

    // save reference to the color sensor
    this.colorSensor = colorSensor;

    // set prev and curr Color to see if color is changing
    prevColor = colorSensor.getColor();
    currColor = colorSensor.getColor();
  }

  /**
   * Controls LED strips on the robot without parameter for ColorSensor.
   */
  public LED () {
    led = new AddressableLED(0); // must be a PWM port
                                 // currently port 0 for testing on miniBot
    ledBuffer = new AddressableLEDBuffer(length);
    led.setLength(length);

    // save reference to the color sensor
    this.colorSensor = new ColorSensor();

    // set prev and curr Color to see if color is changing
    prevColor = colorSensor.getColor();
    currColor = colorSensor.getColor();

    setColor(Color2.kBlack, 0);
  }
 
  /**
   * Sets color of individual pixel
   * @param index index of pixel to be changed
   * @param intensity multiplier for brightness (0.5 is half brightness)
   * @param color String name of color, case sensitive
   */
  public void setColor(int index, double intensity, String color) {
    if (color.equals("Yellow")) ledBuffer.setRGB(index, (int)(255*intensity), (int)(255*intensity), 0);
    else if (color.equals("Red")) ledBuffer.setRGB(index, (int)(255*intensity), 0, 0);
    else if (color.equals("Green")) ledBuffer.setRGB(index, 0, (int)(255*intensity), 0);
    else if (color.equals("Blue")) ledBuffer.setRGB(index, 0, 0, (int)(255*intensity));
    else if (color.equals("Cyan")) ledBuffer.setRGB(index, 0, (int)(255*intensity), (int)(255*intensity));
    else if (color.equals("Pink")) ledBuffer.setRGB(index, (int)(255*intensity), 0, (int)(255*intensity));
    else if (color.equals("Purple")) ledBuffer.setRGB(index, (int)(94*intensity), 0, (int)(255*intensity));
    else if (color.equals("Grey")) ledBuffer.setRGB(index, (int)(111*intensity), (int)(111*intensity), (int)(111*intensity));
    else if (color.equals("White")) ledBuffer.setRGB(index, (int)(255*intensity), (int)(255*intensity), (int)(255*intensity));
    else if (color.equals("Orange")) ledBuffer.setRGB(index, (int)(255*intensity), (int)(102*intensity), 0);
    else if (color.equals("Turquoise")) ledBuffer.setRGB(index, 0, (int)(255*intensity), (int)(128*intensity));
    else ledBuffer.setRGB(index, 0, 0, 0);
    ledBuffer.setRGB(index, 255, 0, 0);
   //System.out.println(index);
  }

  /**
   * Sets entire strip to be one color
   * @param color String name of color, case sensitive
   */
  public void setStrip(String color) {
    for (int i = 0; i < length; i++) {
      setColor(i, 0.5, color);
    }
    led.setData(ledBuffer);
    led.start();
  }

  /**
   * Sets entire strip to be one color
   * @param color String name of color, case sensitive
   * @param brightness is the multiplier for brightness (0.5 is half) 
   */
  public void setStrip(String color, double brightness) {
    for (int i = 0; i < length; i++) {
      setColor(i, brightness, color);
    }
    led.setData(ledBuffer);
    led.start();
  }

  /**
   * Light up the LED strip in a solid color.
   * @param color Color2 to set to the entire strip
   * @param intensity 0 to 1
   */
  public void setColor(Color2 color, double intensity) {
    // int r, g, b;
    for(int l = 0; l < length; l++){
      ledBuffer.setRGB(l, (int)(255*intensity*color.red), (int)(255*intensity*color.green), (int)(255*intensity*color.blue));
    }
    led.setData(ledBuffer);
    led.start();
  }

  /**
   * Light up the LED strip in a pattern.
   * @param pattern 1D array of Color values for the pattern
   * @param intensity 0 to 1
   */
  public void setPattern(Color2[] pattern, double intensity) {
    // int r, g, b;
    for(int l = 0; l < pattern.length; l++){
      ledBuffer.setRGB(l, (int)(255*intensity*pattern[l].red), (int)(255*intensity*pattern[l].green), (int)(255*intensity*pattern[l].blue));
    }
    led.setData(ledBuffer);
    led.start();
  }

  /**
   * turns strip on or off
   * @param on true = on, false = off
   */
  public void setStripState(boolean on) {
    if (on) led.start();
    else led.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    prevColor = currColor;
    currColor = colorSensor.getColor();

    // change color of LED Strip if ColorSensor reads a new color
    if (!prevColor.equals(currColor)) setStrip(currColor);
    
    // ColorSensor updated values on dashboard
    SmartDashboard.putBoolean("Yellow", currColor == ("Yellow"));
    SmartDashboard.putBoolean("Red", (currColor=="Red"));
    SmartDashboard.putBoolean("Green", (currColor == "Green"));
    SmartDashboard.putBoolean("Blue", (currColor == "Blue"));
    SmartDashboard.putString("Color", currColor);
    SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
  }
}
