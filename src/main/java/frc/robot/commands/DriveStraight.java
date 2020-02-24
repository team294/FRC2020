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
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

import static frc.robot.Constants.DriveConstants.*;

public class DriveStraight extends CommandBase {
  /**
   * Uses wpilib TrapezoidProfile generator to generate a motion profile for drive train turning
   * Does not regenerate the profile every time
   */

  private DriveTrain driveTrain; // reference to driveTrain
  private double target; // how many more degrees to the right to turn
  private double maxVel; // max velocity, between 0 and kMaxSpeedMetersPerSecond in Constants 
  private double maxAccel; // max acceleration, between 0 and kMaxAccelerationMetersPerSecondSquared in Constants
  private long profileStartTime; // initial time (time of starting point)
  private long currProfileTime;
  private double targetVel; // velocity to reach by the end of the profile in deg/sec (probably 0 deg/sec)
  private double targetAccel;
  private double startDistLeft;
  private double startDistRight;
  private double endTime;
  private double currDist;
  private double currDistLeft;
  private double currDistRight;
  private double timeSinceStart;
  private boolean regenerate;
  private boolean fromShuffleboard;
  private FileLog log;

  private double aFF;

  private int accuracyCounter = 0;

  private TrapezoidProfileBCR tProfile; // wpilib trapezoid profile generator
  private TrapezoidProfileBCR.State tStateCurr; // initial state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.State tStateNext; // next state of the system as calculated by the profile generator
  private TrapezoidProfileBCR.State tStateFinal; // goal state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.Constraints tConstraints; // max vel (deg/sec) and max accel (deg/sec/sec) of the system

  /**
   * @param target distance to travel, in meters
   * @param maxVel max velocity in meters/second, between 0 and kMaxSpeedMetersPerSecond in Constants
   * @param maxAccel max acceleration in meters/second2, between 0 and kMaxAccelerationMetersPerSecondSquared in Constants
   * @param regenerate true = regenerate profile each cycle (to accurately reach target distance), false = don't regenerate (for debugging)
   * @param driveTrain reference to the drive train subsystem
   * @param log
   */
  public DriveStraight(double target, double maxVel, double maxAccel, boolean regenerate, DriveTrain driveTrain, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.log = log;
    this.regenerate = regenerate;
    this.fromShuffleboard = false;
    this.target = target;
    this.maxVel = MathUtil.clamp(Math.abs(maxVel), 0, DriveConstants.kMaxSpeedMetersPerSecond);
    this.maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, DriveConstants.kMaxAccelerationMetersPerSecondSquared);
    addRequirements(driveTrain);

    aFF = 0.0;
  }

    /**
   * Use this constructor when reading values from Shuffleboard
   * @param regenerate true = regenerate profile each cycle (to accurately reach target distance), false = don't regenerate (for debugging)
   * @param driveTrain reference to the drive train subsystem
   * @param log
   */
  public DriveStraight(boolean regenerate, DriveTrain driveTrain, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.log = log;
    this.regenerate = regenerate;
    this.fromShuffleboard = true;
    this.target = 0;
    this.maxVel = 0.5 * DriveConstants.kMaxSpeedMetersPerSecond;
    this.maxAccel = 0.5 * DriveConstants.kMaxAccelerationMetersPerSecondSquared;
    addRequirements(driveTrain);

    if(SmartDashboard.getNumber("DriveStraight Manual Target Dist", -9999) == -9999) {
      SmartDashboard.putNumber("DriveStraight Manual Target Dist", 2);
    }
    if(SmartDashboard.getNumber("DriveStraight Manual MaxVel", -9999) == -9999) {
      SmartDashboard.putNumber("DriveStraight Manual MaxVel", kMaxSpeedMetersPerSecond);
    }
    if(SmartDashboard.getNumber("DriveStraight Manual MaxAccel", -9999) == -9999) {
      SmartDashboard.putNumber("DriveStraight Manual MaxAccel", kMaxAccelerationMetersPerSecondSquared);
    }

    aFF = 0.0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setTalonPIDConstants(kPLinear, kILinear, kDLinear, 0);

    driveTrain.setDriveModeCoast(false);

    if(fromShuffleboard) {
      target = SmartDashboard.getNumber("DriveStraight Manual Target Dist", 2);
      maxVel = SmartDashboard.getNumber("DriveStraight Manual MaxVel", kMaxSpeedMetersPerSecond);
      maxVel = MathUtil.clamp(Math.abs(maxVel), 0, DriveConstants.kMaxSpeedMetersPerSecond);
      maxAccel = SmartDashboard.getNumber("DriveStraight Manual MaxAccel", kMaxAccelerationMetersPerSecondSquared);
      maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, DriveConstants.kMaxAccelerationMetersPerSecondSquared);
    }

    tStateFinal = new TrapezoidProfileBCR.State(target, 0.0); // initialize goal state (degrees to turn)
    tStateCurr = new TrapezoidProfileBCR.State(0.0, 0.0); // initialize initial state (relative turning, so assume initPos is 0 degrees)

    tConstraints = new TrapezoidProfileBCR.Constraints(maxVel, maxAccel); // initialize velocity
                                                                                                                          // and accel limits
    tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr); // generate profile
    log.writeLog(false, "DriveStraight", "init", "Profile total time", tProfile.totalTime());
    
    endTime = tProfile.totalTime();
    profileStartTime = System.currentTimeMillis(); // save starting time of profile
    currProfileTime = profileStartTime;
    startDistLeft = Units.inchesToMeters(driveTrain.getLeftEncoderInches());
    startDistRight = Units.inchesToMeters(driveTrain.getRightEncoderInches());
    
    aFF = 0.0;
    driveTrain.setTalonPIDConstants(kPLinear, kILinear, kDLinear, 0);
    driveTrain.resetTalonPIDs();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currProfileTime = System.currentTimeMillis();
    currDistLeft = Units.inchesToMeters(driveTrain.getLeftEncoderInches()) - startDistLeft;
    currDistRight = Units.inchesToMeters(driveTrain.getRightEncoderInches()) - startDistRight;
    currDist = (currDistLeft + currDistRight) * 0.5;

    timeSinceStart = (double)(currProfileTime - profileStartTime) * 0.001;
    tStateNext = tProfile.calculate(timeSinceStart);

    targetVel = tStateNext.velocity;
    targetAccel = tStateNext.acceleration;
    aFF = (kSLinear * Math.signum(targetVel)) + (targetVel * kVLinear) + (targetAccel * kALinear);

    SmartDashboard.putNumber("pos: ", tStateNext.position);
    SmartDashboard.putNumber("vel: ", targetVel);
    SmartDashboard.putNumber("%: ", aFF);

    // System.out.println("pos: " + tStateNext.position);
    // System.out.println("vel: " + targetVel);
    // System.out.println("V: " + aFF);

    // driveTrain.setLeftMotorOutput(aFF);
    // driveTrain.setRightMotorOutput(aFF);
    driveTrain.setLeftTalonPIDVelocity(Units.metersToInches(targetVel), aFF);
    driveTrain.setRightTalonPIDVelocity(Units.metersToInches(targetVel), aFF, true);

    log.writeLog(false, "DriveStraight", "profile", "posT", tStateNext.position, "velT", targetVel, "accT", targetAccel,
      "posA", (currDist), "posLA", (currDistLeft), "posRA", (currDistRight), 
      "velLA", (Units.inchesToMeters(driveTrain.getLeftEncoderVelocity())), "velRA", (driveTrain.getRightEncoderVelocity()*2.54 / 100), "aFF", aFF,
      "pctOutLA", driveTrain.getLeftOutputPercent(), "VoutNormA", driveTrain.getLeftOutputVoltage()/compensationVoltage, "VbusLA", driveTrain.getLeftBusVoltage(),
      "velRawLA", driveTrain.getLeftEncoderVelocityRaw(), "errRawLA", driveTrain.getTalonLeftClosedLoopError(), 
      "targetRawL", driveTrain.getTalonLeftClosedLoopTarget());

    double linearVel = Units.inchesToMeters(driveTrain.getAverageEncoderVelocity());
    if(regenerate) {
      tStateCurr = new TrapezoidProfileBCR.State(currDist, linearVel);
      tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr);
      profileStartTime = currProfileTime;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setLeftMotorOutput(0);
    driveTrain.setRightMotorOutput(0);
    driveTrain.setDriveModeCoast(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if((startAng + target) - Units.inchesToMeters(driveTrain.getAverageDistance()) < 0.1) {
    //   System.out.println("Start: " + startAng);
    //   System.out.println("theoretical: " + (startAng + target));
    //   System.out.println("actual: " + Units.inchesToMeters(driveTrain.getAverageDistance()));
    //   return true;
    // }
    if(Math.abs(target - currDist) < 0.0125) {
      accuracyCounter++;
      // System.out.println("theoretical: " + target);
      // System.out.println("actual: " + currDist);
      // System.out.println(accuracyCounter);
      log.writeLog(false, "DriveStraight", "WithinTolerance", "Target Dist", target, "Actual Dist", currDist, "Counter", accuracyCounter);
    } else {
      accuracyCounter = 0;
    }

    return (accuracyCounter >= 5);
  }
}