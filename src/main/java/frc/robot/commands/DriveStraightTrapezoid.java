/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;

import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

import static frc.robot.Constants.DriveConstants.*;

public class DriveStraightTrapezoid extends CommandBase {
  /**
   * Uses wpilib TrapezoidProfile generator to generate a motion profile for drive train turning
   */

  private DriveTrain driveTrain; // reference to driveTrain
  private double target; // how many more degrees to the right to turn
  private double maxVelMultiplier; // multiplier between 0.0 and 1.0 for limiting max velocity
  private double maxAccelMultiplier; // multiplier between 0.0 and 1.0 for limiting max acceleration
  private long profileStartTime; // initial time (time of starting point)
  private double targetVel; // velocity to reach by the end of the profile in deg/sec (probably 0 deg/sec)
  private double targetVoltage; // voltage to pass to the motors to follow profile
  private double startAng; // initial angle (in degrees) (starts as 0 deg for relative turns)
  private double targetAng; // target angle to the right (in degrees)
  private double prevAng;
  private double currAng;
  private double angVel;
  private double endTime;
  private double timeSinceStart;
  private long currUpdateTime;
  private long prevUpdateTime;
  private double changeInTime;
  private FileLog log;


  private TrapezoidProfile tProfile; // wpilib trapezoid profile generator
  private TrapezoidProfile.State tStateCurr; // initial state of the system (position in deg and time in sec)
  private TrapezoidProfile.State tStateNext; // next state of the system as calculated by the profile generator
  private TrapezoidProfile.State tStateFinal; // goal state of the system (position in deg and time in sec)
  private TrapezoidProfile.Constraints tConstraints; // max vel (deg/sec) and max accel (deg/sec/sec) of the system

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
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tStateFinal = new TrapezoidProfile.State(target, 0.0); // initialize goal state (degrees to turn)
    tStateCurr = new TrapezoidProfile.State(0.0, 0.0); // initialize initial state (relative turning, so assume initPos is 0 degrees)

    tConstraints = new TrapezoidProfile.Constraints(kMaxSpeedMetersPerSecond * maxVelMultiplier, kMaxAccelerationMetersPerSecondSquared * maxAccelMultiplier); // initialize velocity
                                                                                                                          // and accel limits
    tProfile = new TrapezoidProfile(tConstraints, tStateFinal, tStateCurr); // generate profile
    System.out.println(tProfile.totalTime());
    
    endTime = tProfile.totalTime();
    profileStartTime = System.currentTimeMillis(); // save starting time of profile
    prevUpdateTime = profileStartTime;
    currUpdateTime = profileStartTime;
    
    startAng = Units.inchesToMeters(driveTrain.getAverageDistance());
    // prevAng = startAng;
    // targetAng = startAng + target; // calculate final angle based on how many degrees are to be turned
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currUpdateTime = System.currentTimeMillis();
    changeInTime = ((double)(currUpdateTime - prevUpdateTime)) / 1000.0;
    // System.out.println(changeInTime);

    timeSinceStart = (double)(currUpdateTime - profileStartTime) / 1000.0;
    tStateNext = tProfile.calculate(changeInTime);

    prevUpdateTime = currUpdateTime;
    
    targetVel = tStateNext.velocity;
    targetVoltage = targetVel * kVAngular;
    // targetVoltage = (targetVoltage < 0.11) ? 0.11 : targetVoltage;
    // targetVoltage = (targetVoltage > 0.5) ? 0.5 : targetVoltage;


    SmartDashboard.putNumber("pos: ", tStateNext.position);
    SmartDashboard.putNumber("vel: ", targetVel);
    SmartDashboard.putNumber("V: ", targetVoltage);

    log.writeLog(false, "DriveStraight", "profile", "posT", tStateNext.position, "velT", targetVel, 
    "posA", (Units.inchesToMeters(driveTrain.getAverageDistance()) - startAng), 
    "velLA", (driveTrain.getLeftEncoderVelocity()*2.52 / 100), "velRA", (driveTrain.getRightEncoderVelocity()*2.52 / 100), "V", targetVoltage);
    System.out.println("pos: " + tStateNext.position);
    System.out.println("vel: " + targetVel);
    System.out.println("V: " + targetVoltage);

    driveTrain.setRightMotorOutput(-targetVoltage);
    driveTrain.setLeftMotorOutput(targetVoltage);

    // currAng = Units.inchesToMeters(driveTrain.getAverageDistance()) - startAng;
    // angVel = driveTrain.getAverageEncoderVelocity() * 2.54 / 100;
    // System.out.println(currAng);
    // System.out.println(angVel);
    // tStateCurr = new TrapezoidProfile.State(currAng, angVel);
    tProfile = new TrapezoidProfile(tConstraints, tStateFinal, tStateNext);
    // prevAng = currAng;
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
    if(timeSinceStart >= endTime) {
      System.out.println("Start: " + startAng);
      System.out.println("theoretical: " + (startAng + target));
      System.out.println("actual: " + Units.inchesToMeters(driveTrain.getAverageDistance()));
      return true;
    }
    return false;
  }
}