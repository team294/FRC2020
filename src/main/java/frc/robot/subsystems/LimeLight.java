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
import frc.robot.Constants.LimeLightConstants;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.RobotPreferences;
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
  private DriveTrain driveTrain; // for testing distance calculation, probs can be taken out dist calc finished
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

  public LimeLight(FileLog log, LED led, DriveTrain driveTrain) {
    this.log = log;
    this.led = led;
    this.driveTrain = driveTrain;
    tableInstance.startClientTeam(294);

    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    pipeline = table.getEntry("pipeline");
    SmartDashboard.putNumber("Pipeline", 0);

  }

  /**
   * @return horizontal (x-axis) angle, in degrees, between camera crosshair and target crosshair
   * + = target is to the left, - = target is to the right
   */
  public double getXOffset() {
    return x;
  }

  /**
   * @return vertical (y-axis) angle, in degrees, between camera crosshair and target crosshair
   * down is negative, up is positive
   */
  public double getYOffset() {
    return y;
  }

  public double getArea() {
    return area;
  }
  public double getSweetSpot() {
    double sweetSpotDistance;
    sweetSpotDistance = getDistanceNew() - endDistance;
    SmartDashboard.putNumber("Sweet spot distance", sweetSpotDistance);
    System.out.println("sweet spot distance " + sweetSpotDistance);
    return sweetSpotDistance;
  }
  /**
   * @return distance, on the floor, from camera to target
   * takes into account not being in line with the target
   */
  public double getDistanceNew(){
    double myDistance = (targetHeight-cameraHeight)/((Math.tan(Math.toRadians(cameraAngle + y)))*(Math.cos(Math.toRadians(x))));
    return myDistance;
  }

  /**
   * @return distance, on the floor, from camera to target
   * assumes camera is perfectly in line with the target
   * used for preliminary distanceCalc tests
   * DO NOT USE FOR ACTUAL DISTANCE CALCULATIONS, USE NEW GETDIST
   */
  public double getDistance() {
    double myDistance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(cameraAngle+y));
    return myDistance;
  }

  public double getPipeline() {
    return table.getEntry("getpipe").getDouble(0);
  }

  /**
   * @param pipeNum 0 is vision, 2 is driver cam
   */
  public void setPipe(double pipeNum) {
    pipeline.setDouble(pipeNum);
  }

  /**
   * chooses which pattern to display based on the x offset value from limelight
   * returns Color2[] array
   */
  public Color[] makePattern() {
    Color[] myPattern = new Color[16];
    int patternFormula = (int) ((-x + 7));
    if (patternFormula < 0) {
      patternFormula = 0;
    } else if (patternFormula > 14) {
      patternFormula = 14;
    }
    myPattern = LED.patternLibrary[patternFormula];
    if (!seesTarget()) {
      myPattern = LED.patternLibrary[15];
    }
    return myPattern;
  }

  /**
   * @return true when limelight sees a target, false when not seeing a target
   */
  public boolean seesTarget() {
    return (x != 0 && x != 1064);
  }

  /**
   * @return true when limelight is connected & reading data
   * false when limelight is disconnected or not reading any data
   */
  public boolean isGettingData() {
    return (x != 1064 && y != 1000);
  }

  @Override
  public void periodic() {
    // table.addEntryListener(Value."tl".name, this::updateValues, kNew | kUpdate);

    // read values periodically
    x = -tx.getDouble(1000.0) * LimeLightConstants.angleMultiplier;
    y = ty.getDouble(1000.0);
    area = ta.getDouble(1000.0);
    theoreticalWidth = Math.sqrt(area) * 1.526;

    if (makePattern() == LED.patternLibrary[15]) {
      led.setPattern(makePattern(), 0.1, 0);
    } else {
      led.setPattern(makePattern(), 0.5, 0);
    }

    // led.setPattern(ledAnimation.getNextPattern(), 0.5, 1);
    // ledAnimation.setDelayCounter();

    if (log.getLogRotation() == log.LIMELIGHT_CYCLE) {
      updateLimeLightLog(false);

      if(!isGettingData()) {
        RobotPreferences.recordStickyFaults("LimeLight", log);
      }

      // Invert X on SmartDashboard, since bars on SmartDashboard always go from - (left) to + (right)
      SmartDashboard.putNumber("LimeLight x", -x);
      SmartDashboard.putNumber("LimeLight y", y);
      //SmartDashboard.putNumber("Limelight dist", getDistance()); // distance assuming we are in line with the target
      SmartDashboard.putNumber("Limelight new distance", getDistanceNew()); // distance calculation using vision camera
      SmartDashboard.putNumber("Limelight Actual dist", (-driveTrain.getAverageDistance()/12)); // distance calculation using drive encoders, used to test accuracy of getDistanceNew()
      SmartDashboard.putBoolean("Limelight Updating", isGettingData());
      SmartDashboard.putBoolean("Limelight Sees Target", seesTarget());
      pipe = SmartDashboard.getNumber("Pipeline", 0); // default is vision pipeline

      if (getPipeline() != pipe) {
        log.writeLogEcho(false, "LimeLight", "Pipeline change", "Pipeline", pipe);
        setPipe(pipe);
      }
    }
  }

  /**
   * Write information about limelight to fileLog.
   * 
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  public void updateLimeLightLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, "LimeLight", "Update Variables", 
      "Center Offset X", x, 
      "Center Offset Y", y,
      "Target Area", area,
      "Dist", getDistance(), "New Dist", getDistanceNew(), "Actual Dist", (-driveTrain.getAverageDistance()/12)
      );
  }
}