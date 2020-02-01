/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

import frc.robot.subsystems.*;
import frc.robot.utilities.*;

import static frc.robot.Constants.DriveConstants.*;

public class DriveStraightTrapezoid extends CommandBase {
  /**
   * Uses wpilib TrapezoidProfile generator to generate a motion profile for drive train turning
   * Does not regenerate the profile every time
   */

  private DriveTrain driveTrain; // reference to driveTrain
  private double target; // how many more degrees to the right to turn
  private double maxVelMultiplier; // multiplier between 0.0 and 1.0 for limiting max velocity
  private double maxAccelMultiplier; // multiplier between 0.0 and 1.0 for limiting max acceleration
  private long profileStartTime; // initial time (time of starting point)
  private long currProfileTime;
  private double targetVel; // velocity to reach by the end of the profile in deg/sec (probably 0 deg/sec)
  private double targetAccel;
  private double startDist; // initial angle (in degrees) (starts as 0 deg for relative turns)
  private double endTime;
  private double timeSinceStart;
  private FileLog log;

  private double kP;
  private double kI;
  private double kD;
  private double aFF;

  private int accuracyCounter = 0;

  private TrapezoidProfileBCR tProfile; // wpilib trapezoid profile generator
  private TrapezoidProfileBCR.State tStateCurr; // initial state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.State tStateNext; // next state of the system as calculated by the profile generator
  private TrapezoidProfileBCR.State tStateFinal; // goal state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.Constraints tConstraints; // max vel (deg/sec) and max accel (deg/sec/sec) of the system

  /**
   * @param driveTrain reference to the drive train subsystem
   * @param target degrees to turn to the right
   * @param maxVelMultiplier between 0.0 and 1.0, multipier for limiting max velocity
   * @param maxAccelMultiplier between 0.0 and 1.0, multiplier for limiting max acceleration
   */
  public DriveStraightTrapezoid(DriveTrain driveTrain, FileLog log, double target, double maxVelMultiplier, double maxAccelMultiplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.log = log;
    this.target = target;
    this.maxVelMultiplier = maxVelMultiplier;
    this.maxAccelMultiplier = maxAccelMultiplier;

    addRequirements(driveTrain);

    kP = 0.0;
    kI = 0.0;
    kD = 0.0;
    aFF = 0.0;

    driveTrain.setTalonPIDConstants(kP, kI, kD, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tStateFinal = new TrapezoidProfileBCR.State(target, 0.0); // initialize goal state (degrees to turn)
    tStateCurr = new TrapezoidProfileBCR.State(0.0, 0.0); // initialize initial state (relative turning, so assume initPos is 0 degrees)

    tConstraints = new TrapezoidProfileBCR.Constraints(kMaxSpeedMetersPerSecond * maxVelMultiplier, kMaxAccelerationMetersPerSecondSquared * maxAccelMultiplier); // initialize velocity
                                                                                                                          // and accel limits
    tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr); // generate profile
    System.out.println(tProfile.totalTime());
    
    endTime = tProfile.totalTime();
    profileStartTime = System.currentTimeMillis(); // save starting time of profile
    currProfileTime = profileStartTime;
    startDist = Units.inchesToMeters(driveTrain.getLeftEncoderInches()); //getAverageDistance());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currProfileTime = System.currentTimeMillis();
    timeSinceStart = (double)(currProfileTime - profileStartTime) / 1000.0;
    tStateNext = tProfile.calculate(timeSinceStart);
    targetVel = tStateNext.velocity;
    targetAccel = tStateNext.acceleration;
    aFF = (kSLinear * Math.signum(targetVel)) + (targetVel * kVLinear) + (targetAccel * kALinear);
    System.out.println(targetVel);

    SmartDashboard.putNumber("pos: ", tStateNext.position);
    SmartDashboard.putNumber("vel: ", targetVel);
    SmartDashboard.putNumber("%: ", aFF);

    log.writeLog(false, "DriveStraight", "profile", "posT", tStateNext.position, "velT", targetVel, 
    "posA", (Units.inchesToMeters(driveTrain.getLeftEncoderInches()) - startDist), 
    "velLA", (Units.inchesToMeters(driveTrain.getLeftEncoderVelocity())), "velRA", (driveTrain.getRightEncoderVelocity()*2.54 / 100), "V", aFF);
    System.out.println("pos: " + tStateNext.position);
    System.out.println("vel: " + targetVel);
    System.out.println("V: " + aFF);

    driveTrain.setTalonPIDVelocity(Units.metersToInches(targetVel), aFF, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setLeftMotorOutput(0);
    driveTrain.setRightMotorOutput(0);
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
    if(Math.abs((startDist + target) - Units.inchesToMeters(driveTrain.getLeftEncoderInches())) < 0.0125) {
      accuracyCounter++;
      System.out.println("Start: " + startDist);
      System.out.println("theoretical: " + (startDist + target));
      System.out.println("actual: " + Units.inchesToMeters(driveTrain.getLeftEncoderInches()));
      System.out.println(accuracyCounter);
    } else {
      accuracyCounter = 0;
    }

    if(accuracyCounter >= 5) {
      return true;
    }
    return false;
  }
}