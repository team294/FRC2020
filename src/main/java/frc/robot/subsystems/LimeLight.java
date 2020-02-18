/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.utilities.FileLog;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.LED;
import static frc.robot.Constants.LimeLightConstants.*;

public class LimeLight extends SubsystemBase {

  private static NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
  private static NetworkTable table = tableInstance.getTable("limelight");
  private NetworkTableEntry tv, tx, ty, ta, pipeline;
  public double x, y, area;
  private FileLog log;
  private LED led;
  private double pipe;
  private double theoreticalWidth;

  /*
   * Limelight settings: ~~input~~ Exposure: 2 Black level offset: 0 red balance:
   * 1200 blue balance: 1975 ~~thresholding~~ hue: 60 - 95 saturation: 150 - 255
   * value: 170 - 255 erosion steps: 0 dilation steps: 0 ~~countour filtering~~
   * area % image: 0.0163 - 100 fullness % of blue rectangle: 10 - 100 w/h ratio:
   * 0 - 20 direction filter : none smart specle detection: 0 target grouping:
   * single target intersection filter: none ~~output~~ targeting region: center
   * send raw corners? : nah send raw contours: no crosshair mode: Single
   * crosshair x: 0 y: 0 ~~3d experimental~~ no changes
   */

  public LimeLight(FileLog log, LED led) {
    this.log = log;
    this.led = led;
    tableInstance.startClientTeam(294);

    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    pipeline = table.getEntry("pipeline");
    SmartDashboard.putNumber("Pipeline", 2);

  }

  public double getXOffset() {
    return x;
  }

  public double getYOffset() {
    return y;
  }

  public double getArea() {
    return area;
  }

  public double getDistance() {
    double myDistance = (7 - 1.583) / Math.tan(Math.toRadians(26.5+y));
    System.out.println(Math.tan(Math.toRadians(27+y)));
    System.out.println(y + " Dist" + myDistance);
   // double myDistance = (-9.8209 * area) + 25.6719;
    return myDistance;
  }

  public double getPipeline() {
    return table.getEntry("getpipe").getDouble(0);
  }

  public void setPipe(double pipeNum) {
    pipeline.setDouble(pipeNum);
  }

  /**
   * chooses which pattern to display based on the x offset value from limelight
   * returns Color2[] array
   */
  public Color[] makePattern() {
    Color[] myPattern = new Color[16];
    int patternFormula = (int) ((x + 7));
    if (patternFormula < 0) {
      patternFormula = 0;
    } else if (patternFormula > 14) {
      patternFormula = 14;
    }
    myPattern = LED.patternLibrary[patternFormula];
    if (x == 0) {
      myPattern = LED.patternLibrary[15];
    }
    return myPattern;
  }

  @Override
  public void periodic() {
    // table.addEntryListener(Value."tl".name, this::updateValues, kNew | kUpdate);

    // read values periodically
    x = tx.getDouble(1000.0) * LimeLightConstants.angleMultiplier;
    y = ty.getDouble(1000.0);
    area = ta.getDouble(1000.0);
    theoreticalWidth = Math.sqrt(area) * 1.526;

    SmartDashboard.putNumber("LimeLight x", x);
    SmartDashboard.putNumber("LimeLight y", y);
    SmartDashboard.putNumber("Limelight dist", getDistance());
    // SmartDashboard.putNumber("LimeLight distance", getDistance());

    pipe = SmartDashboard.getNumber("Pipeline", 2);

    if (getPipeline() != pipe) {
      System.out.println("Pipeline changed: " + pipe);
      setPipe(pipe);
    }

    if (makePattern() == LED.patternLibrary[15]) {
      led.setPattern(makePattern(), 0.1, 0);
    } else {
      led.setPattern(makePattern(), 0.5, 0);
    }

    // led.setPattern(ledAnimation.getNextPattern(), 0.5, 1);
    // ledAnimation.setDelayCounter();

    // updateLimeLightLog(false);

    if (log.getLogRotation() == log.LIMELIGHT_CYCLE) {
      updateLimeLightLog(false);
    }
  }

  /**
   * Write information about limelight to fileLog.
   * 
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  public void updateLimeLightLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, "LimeLight", "Update Variables", "Center Offset X", x, "Center Offset Y", y,
        "Target Area", area
    // "Theoretical width", theoreticalWidth,
    // "Theoretical distance", getDistance()
    );
  }
}