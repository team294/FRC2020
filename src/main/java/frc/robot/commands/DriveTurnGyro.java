/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.PIDController;

import frc.robot.subsystems.*;
import frc.robot.utilities.*;

import static frc.robot.Constants.DriveConstants.*;

public class DriveTurnGyro extends CommandBase {
  /**
   * Uses wpilib TrapezoidProfile generator to generate a motion profile for drive train turning
   * Does not regenerate the profile every time
   */

  private DriveTrain driveTrain; // reference to driveTrain
  private double target; // how many more degrees to the right to turn
  private double direction; // -1 = turn to the left, +1 = turn to the right
  private double maxVel; // max velocity, between 0 and kMaxAngularVelocity in Constants
  private double maxAccel; // max acceleration, between 0 and kMaxAngularAcceleration in Constants
  private long profileStartTime; // initial time (time of starting point)
  private long currProfileTime;
  private double targetVel; // velocity to reach by the end of the profile in deg/sec (probably 0 deg/sec)
  private double targetAccel;
  private double startAngle, targetRel; // starting angle in degrees, target angle relative to start angle
  private double currAngle, currVelocity;
  private double timeSinceStart;
  private TargetType targetType;
  private boolean regenerate;
  private boolean fromShuffleboard;
  private FileLog log;
  private LimeLight limeLight;
  private PIDController pidAngVel;
  private double angleTolerance;

  private double aFF, pFB;  // variables for arbitrary feed forward and feedback power

  private int accuracyCounter = 0;

  private TrapezoidProfileBCR tProfile; // wpilib trapezoid profile generator
  private TrapezoidProfileBCR.State tStateCurr; // initial state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.State tStateNext; // next state of the system as calculated by the profile generator
  private TrapezoidProfileBCR.State tStateFinal; // goal state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.Constraints tConstraints; // max vel (deg/sec) and max accel (deg/sec/sec) of the system

  public enum TargetType {
    kRelative(0),
    kAbsolute(1),
    kVision(2);

    @SuppressWarnings({"MemberName", "PMD.SingularField"})
    public final int value;

    TargetType(int value) { this.value = value; }
  }

  /**
   * Turns the robot to a target angle.
   * @param type kRelative (target is an angle relative to current robot facing),
   *   kAbsolute (target is an absolute field angle; 0 = away from drive station),
   *   kVision (use limelight to turn towards the goal)
   * @param maxVelMultiplier between 0.0 and 1.0, multipier for limiting max velocity
   * @param maxAccelMultiplier between 0.0 and 1.0, multiplier for limiting max acceleration
   * @param regenerate true to regenerate profile while running
   * @param angleTolerance the tolerance to use for turn gyro
   * @param driveTrain drivetrain
   * @param limeLight limelight
   * @param log log
   */
  public DriveTurnGyro(TargetType type, double target, double maxVelMultiplier, double maxAccelMultiplier, boolean regenerate, double angleTolerance, DriveTrain driveTrain, LimeLight limeLight, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.limeLight = limeLight;
    this.log = log;
    this.target = driveTrain.normalizeAngle(target);
    this.maxVel = maxVelMultiplier;
    this.maxAccel = maxAccelMultiplier;
    this.targetType = type;
    this.regenerate = regenerate;
    this.fromShuffleboard = false;
    this.angleTolerance = angleTolerance;

    addRequirements(driveTrain, limeLight);

    aFF = 0.0;

    pidAngVel = new PIDController(kPAngular, kIAngular, kDAngular);
  }

  /**
   * To be used when changing the target value directly from shuffleboard (not a pre-coded target)
   * @param fromShuffleboard true means the value is being changed from shuffleboard
   */
  
  /**
   * Turns the robot to a target angle.
   * @param type kRelative (target is an angle relative to current robot facing),
   *   kAbsolute (target is an absolute field angle; 0 = away from drive station),
   *   kVision (use limelight to turn towards the goal)
   * @param target degrees to turn from +180 (left) to -180 (right) [ignored for kVision]
   * @param maxVelMultiplier between 0.0 and 1.0, multipier for limiting max velocity
   * @param maxAccelMultiplier between 0.0 and 1.0, multiplier for limiting max acceleration
   * @param angleTolerance the tolerance to use for turn gyro
   * @param driveTrain drivetrain
   * @param limeLight limelight
   * @param log log
   */
  public DriveTurnGyro(TargetType type, double target, double maxVelMultiplier, double maxAccelMultiplier, double angleTolerance, DriveTrain driveTrain, LimeLight limeLight, FileLog log) {
    this(type, target, maxVelMultiplier, maxAccelMultiplier, true, angleTolerance, driveTrain, limeLight, log);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setDriveModeCoast(true);

    if(fromShuffleboard) {
      target = SmartDashboard.getNumber("TurnGyro Manual Target Ang", 90);
      maxVel = SmartDashboard.getNumber("TurnGyro Manual MaxVel", kMaxAngularVelocity);
      maxAccel = SmartDashboard.getNumber("TurnGyro Manual MaxAccel", kMaxAngularAcceleration);
    }

    startAngle = driveTrain.getGyroRotation();

    switch (targetType) {
      case kRelative:
        targetRel = target;
        break;
      case kAbsolute:
        targetRel = driveTrain.normalizeAngle(target - startAngle);
        break;
      case kVision:
        targetRel = driveTrain.normalizeAngle(limeLight.getXOffset());
        break;
    }

    direction = Math.signum(targetRel);

    tStateFinal = new TrapezoidProfileBCR.State(targetRel, 0.0); // initialize goal state (degrees to turn)
    tStateCurr = new TrapezoidProfileBCR.State(0.0, 0.0); // initialize initial state (relative turning, so assume initPos is 0 degrees)

    // initialize velocity and accel limits
    tConstraints = new TrapezoidProfileBCR.Constraints(maxVel, maxAccel);
    // generate profile
    tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr);

    profileStartTime = System.currentTimeMillis(); // save starting time of profile
    currProfileTime = profileStartTime;

    pidAngVel.reset();

    log.writeLog(false, "DriveTurnGyro", "initialize", "Total Time", tProfile.totalTime(), "StartAngleAbs", startAngle, "TargetAngleRel", targetRel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currProfileTime = System.currentTimeMillis();
    // currAngle is relative to the startAngle.  +90 to -270 if turning left, -90 to +270 if turning right.
    currAngle = driveTrain.normalizeAngle(driveTrain.getGyroRotation() - startAngle);
    currAngle += (direction*currAngle<-90) ? direction*360.0 : 0; 
    currVelocity = driveTrain.getAngularVelocity();
    
    if (targetType == TargetType.kVision) {
      targetRel = driveTrain.normalizeAngle(currAngle + limeLight.getXOffset());
      tStateFinal = new TrapezoidProfileBCR.State(targetRel, 0.0);
    }

    timeSinceStart = (double)(currProfileTime - profileStartTime) * 0.001;
    tStateNext = tProfile.calculate(timeSinceStart + 0.010);

    targetVel = tStateNext.velocity;
    targetAccel = tStateNext.acceleration;
    aFF = (kSAngular * Math.signum(targetVel)) + (targetVel * kVAngular) + (targetAccel * kAAngular);

    // SmartDashboard.putNumber("TurnGyro target angle", tStateNext.position);

    pFB = MathUtil.clamp(pidAngVel.calculate(currVelocity, targetVel), -0.1, 0.1);
    //pFB = 0; 

    driveTrain.setLeftMotorOutput(-aFF - pFB);
    driveTrain.setRightMotorOutput(aFF + pFB);

    if (regenerate) {
      tStateCurr = new TrapezoidProfileBCR.State(currAngle, targetVel);
      tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr);
      profileStartTime = currProfileTime;
    }

    log.writeLog(false, "DriveTurnGyro", "profile", "posT", tStateNext.position, "velT", targetVel, "accT", targetAccel,
      "posA", currAngle, "velA", currVelocity, "aFF", aFF, "pFB", pFB, "pTotal", aFF+pFB, "LL x", limeLight.getXOffset(), "LL y", limeLight.getYOffset());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setLeftMotorOutput(0);
    driveTrain.setRightMotorOutput(0);
    driveTrain.setDriveModeCoast(false);

    log.writeLog(false, "DriveTurnGyro", "end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(targetRel - currAngle) < angleTolerance) {
      accuracyCounter++;
      // System.out.println("theoretical: " + targetRel);
      // System.out.println("actual: " + currAngle);
      // System.out.println(accuracyCounter);
    } else {
      accuracyCounter = 0;
    }

    if(accuracyCounter >= 5) {
      return true;
    }
    return false;
  }
}