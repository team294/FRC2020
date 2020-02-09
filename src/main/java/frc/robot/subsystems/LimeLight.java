/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utilities.FileLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utilities.*;
import frc.robot.subsystems.LED;

public class LimeLight extends SubsystemBase {

  private static NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
  private static NetworkTable table = tableInstance.getTable("limelight");
  public double x, y, area;
  private FileLog log;
  private LED led2;
  /*
  Limelight settings:
  ~~input~~
  Exposure: 2
  Black level offset: 0
  red balance: 1200
  blue balance: 1975
  ~~thresholding~~
  hue:  60 - 95
  saturation: 150 - 255
  value: 170 - 255
  erosion steps: 0
  dilation steps: 0
  ~~countour filtering~~
  area % image: 0.0163 - 100
  fullness % of blue rectangle: 10 - 100
  w/h ratio: 0 - 20
  direction filter : none
  smart specle detection: 0
  target grouping: single target
  intersection filter: none
  ~~output~~
  targeting region: center
  send raw corners? : nah
  send raw contours: no
  crosshair mode: Single crosshair
  x: 0
  y: 0
  ~~3d experimental~~
  no changes 
  */

  public LimeLight(FileLog log, LED led2) {
    this.log = log;
    this.led2 = led2;
    tableInstance.startClientTeam(294);

  }

  public double getXOffset() {
    return x;
  }

  public double getYOffset() {
    return y;
  }
  
  public Color2[] makePattern(){
    Color2[] myPattern = new Color2[16];
    int patternNumber = (int)(x/3.6) + 2;
    
    myPattern = LED.patternLibrary[patternNumber];
    return myPattern;
  }
  

  @Override
  public void periodic() {
    // table.addEntryListener(Value."tl".name, this::updateValues, kNew | kUpdate);

    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    // read values periodically
    x = tx.getDouble(1000.0);
    y = ty.getDouble(1000.0);
    area = ta.getDouble(1000.0);

    SmartDashboard.putNumber("LimeLight x: ", x);
    SmartDashboard.putNumber("LimeLight y: ", y);
    updateLimeLightLog(false);

    led2.setStrip("Red");
  }

   /**
   * Write information about limelight to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
	public void updateLimeLightLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, "LimeLight", "Update Variables",  
    "Center Offset X", x,
    "Center Offset Y", y,
    "Target Area", area
    );
  }
}