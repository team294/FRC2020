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
  private double direction;     // -1 = turn to the left, +1 = turn to the right
  private double maxVelMultiplier; // multiplier between 0.0 and 1.0 for limiting max velocity
  private double maxAccelMultiplier; // multiplier between 0.0 and 1.0 for limiting max acceleration
  private long profileStartTime; // initial time (time of starting point)
  private long currProfileTime;
  private double targetVel; // velocity to reach by the end of the profile in deg/sec (probably 0 deg/sec)
  private double targetAccel;
  private double startAngle; // starting angle in degrees
  private double currAngle, currVelocity;
  private double timeSinceStart;
  private boolean useVision,useAbsoloteAngle, regenerate;
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

  /**
   * 
   ** @param target degrees to turn from -180 (left) to 180 (right) relative to current orientation
   * @param maxVelMultiplier between 0.0 and 1.0, multipier for limiting max velocity
   * @param maxAccelMultiplier between 0.0 and 1.0, multiplier for limiting max acceleration
   * @param useVision true to use vison (instead of target angle)
   * @param regenerate true to regenerate profile while running
   * @param useAbsoloteAngle true to use field angles
   * @param angleTolerance the tolerance to use for turn gyro
   * @param driveTrain drivetrain
   * @param limeLight limelight
   * @param log log
   */
  public DriveTurnGyro(double target, double maxVelMultiplier, double maxAccelMultiplier, boolean useVision, boolean regenerate, boolean useAbsoloteAnle, double angleTolerance, DriveTrain driveTrain, LimeLight limeLight, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.limeLight = limeLight;
    this.log = log;
    this.target = driveTrain.normalizeAngle(target);
    this.maxVelMultiplier = maxVelMultiplier;
    this.maxAccelMultiplier = maxAccelMultiplier;
    this.useVision = useVision;
    this.regenerate = regenerate;
    this.useAbsoloteAngle = useAbsoloteAngle;
    this.angleTolerance = angleTolerance;

    addRequirements(driveTrain);

    aFF = 0.0;

    //driveTrain.setTalonPIDConstants(kP, kI, kD, 0);
    pidAngVel = new PIDController(kPAngular, kIAngular, kDAngular);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setDriveModeCoast(true);

    startAngle = driveTrain.getGyroRotation();

    if (useVision) {
      target = driveTrain.normalizeAngle(limeLight.getXOffset());
    }
    else if(useAbsoloteAngle){
      this.target = driveTrain.normalizeAngle(target - startAngle);
    }

    direction = Math.signum(target);

    tStateFinal = new TrapezoidProfileBCR.State(target, 0.0); // initialize goal state (degrees to turn)
    tStateCurr = new TrapezoidProfileBCR.State(0.0, 0.0); // initialize initial state (relative turning, so assume initPos is 0 degrees)

    // initialize velocity and accel limits
    tConstraints = new TrapezoidProfileBCR.Constraints(kMaxAngularVelocity * maxVelMultiplier, kMaxAngularAcceleration * maxAccelMultiplier); 
    // generate profile
    tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr);

    profileStartTime = System.currentTimeMillis(); // save starting time of profile
    currProfileTime = profileStartTime;

    pidAngVel.reset();

    log.writeLog(false, "DriveTurnGyro", "initialize", "Total Time", tProfile.totalTime());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currProfileTime = System.currentTimeMillis();
    // currAngle is relative to the startAngle.  +90 to -270 if turning left, -90 to +270 if turning right.
    currAngle = driveTrain.normalizeAngle((driveTrain.getGyroRotation() - startAngle));
    currAngle += (direction*currAngle<-90) ? direction*360.0 : 0; 
    currVelocity = driveTrain.getAngularVelocity();
    
    if (useVision) {
      target = driveTrain.normalizeAngle(currAngle + limeLight.getXOffset());
      tStateFinal = new TrapezoidProfileBCR.State(target, 0.0);
    }

    timeSinceStart = (double)(currProfileTime - profileStartTime) * 0.001;
    tStateNext = tProfile.calculate(timeSinceStart + 0.010);

    targetVel = tStateNext.velocity;
    targetAccel = tStateNext.acceleration;
    aFF = (kSAngular * Math.signum(targetVel)) + (targetVel * kVAngular) + (targetAccel * kAAngular);

    // SmartDashboard.putNumber("TurnGyro target angle", tStateNext.position);

    pFB = MathUtil.clamp(pidAngVel.calculate(currVelocity, targetVel), -0.1, 0.1);
    //pFB = 0; 

    driveTrain.setLeftMotorOutput(aFF + pFB);
    driveTrain.setRightMotorOutput(-aFF - pFB);

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
    if(Math.abs(target - currAngle) < angleTolerance) {
      accuracyCounter++;
      // System.out.println("theoretical: " + target);
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