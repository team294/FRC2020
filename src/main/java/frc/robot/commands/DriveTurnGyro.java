/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;
import frc.robot.utilities.*;

import static frc.robot.Constants.DriveConstants.*;

public class DriveTurnGyro extends CommandBase {
  /**
   * Uses wpilib TrapezoidProfile generator to generate a motion profile for drive train turning
   */

  // references to other classes
  private DriveTrain driveTrain; // reference to driveTrain
  private FileLog log; // reference to the file log

  // kinematics
  private double maxVelMultiplier; // multiplier between 0.0 and 1.0 for limiting max velocity
  private double maxAccelMultiplier; // multiplier between 0.0 and 1.0 for limiting max acceleration
  private double targetVel; // velocity to reach by the end of the profile in deg/sec (probably 0 deg/sec)
  private double targetOutput; // percentOutput to pass to the motors to follow profile, should be between -1.0 and 1.0
  
  // angle values
  private double target; // how many degrees to the right to turn
  private double startAng; // initial angle (in degrees) (starts as 0 deg for relative turns)
  private double targetAng; // target angle to the right (in degrees)
  private double prevAng;
  private double currAng;
  private double angVel;

  // time values
  private double profileEndTime; // time that profile should take to finish
  private double profileRunTime; // time since profile was started
  private long profileStartTime; // initial time (time of starting point)
  private double changeInTime; // time since last scheduler cycle
  private long prevUpdateTime; // time of last scheduler cycle
  private long currUpdateTime; // time of current scheduler cycle

  // Trapezoid profile objects
  private TrapezoidProfileBCR tProfile; // wpilib trapezoid profile generator
  private TrapezoidProfileBCR.State tStateCurr; // initial state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.State tStateNext; // next state of the system as calculated by the profile generator
  private TrapezoidProfileBCR.State tStateFinal; // goal state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.Constraints tConstraints; // max vel (deg/sec) and max accel (deg/sec/sec) of the system

  /**
   * @param driveTrain reference to the drive train subsystem
   * @param log reference to the file log utility
   * @param target degrees to turn to the right
   * @param maxVelMultiplier between 0.0 and 1.0, multipier for limiting max velocity
   * @param maxAccelMultiplier between 0.0 and 1.0, multiplier for limiting max acceleration
   */
  public DriveTurnGyro(DriveTrain driveTrain, FileLog log, double target, double maxVelMultiplier, double maxAccelMultiplier) {
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
    tStateFinal = new TrapezoidProfileBCR.State(target, 0.0); // set goal/target (degrees to turn to the right)
    tStateCurr = new TrapezoidProfileBCR.State(0.0, 0.0); // set inital state (relative turning, so assume initPos is 0 degrees)

    tConstraints = new TrapezoidProfileBCR.Constraints(kMaxAngularVelocity * maxVelMultiplier, kMaxAngularAcceleration * maxAccelMultiplier); // initialize velocity
                                                                                                                                           // and accel limits
    tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr); // generate profile
    
    profileEndTime = tProfile.totalTime(); // save time profile should take to run
    profileStartTime = System.currentTimeMillis(); // save starting time of profile
    prevUpdateTime = profileStartTime; // used to calculate change in time between scheduler cycles
    currUpdateTime = profileStartTime; // used to calculate change in time between scheduler cycles
    
    startAng = driveTrain.getGyroRaw(); // use raw gyro angle to prevent wrap-around
    prevAng = startAng; // used to calculate actual angular velocity
    currAng = startAng; // used to calculate actual angular velocity
    targetAng = startAng + target; // calculate final angle based on how many degrees are to be turned
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currUpdateTime = System.currentTimeMillis();
    // calculate change in time between scheduler cycles
    // type-cast from long to double to not lose data from System.currentTimeMillis (super long number)
    // divide by 1000 to convert from milliseconds to seconds
    changeInTime = ((double)(currUpdateTime - prevUpdateTime)) / 1000.0;
    profileRunTime = (double)(currUpdateTime - profileStartTime) / 1000.0;

    // time parameter for tProfile.calculate is time 
    // since calculate() was last called, in seconds
    // aka change in time between scheduler cycles
    tStateNext = tProfile.calculate(changeInTime);

    targetVel = tStateNext.velocity;
    targetOutput = targetVel * kVAngular; // TODO tune kVAngular

    // calculate actual angular velocity
    currAng = driveTrain.getGyroRaw();
    angVel = (currAng - prevAng) / (changeInTime);

    // write to fileLog, can be removed after testing
    log.writeLog(false, "DriveStraight", "profile", "posT", tStateNext.position, "velT", targetVel, // theoretical values
                                                    "posA", driveTrain.getGyroRaw(), "velA", angVel, // actual values
                                                    "%", targetOutput); // percentOutput passed to motor



    System.out.println("pos: " + tStateNext.position); // can be removed after testing
    System.out.println("vel: " + targetVel); // can be removed after testing
    System.out.println("V: " + targetOutput); // can be removed after testing

    driveTrain.setRightMotorOutput(targetOutput);
    driveTrain.setLeftMotorOutput(targetOutput);

    tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateNext); // update profile with next point

    prevAng = currAng;
    prevUpdateTime = currUpdateTime;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop motors at end of profile
    driveTrain.setLeftMotorOutput(0); 
    driveTrain.setRightMotorOutput(0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(profileRunTime >= profileEndTime) {
      System.out.println("Start Ang: " + startAng); // can be removed after testing
      System.out.println("Theoretical End Ang: " + (startAng + target)); // can be removed after testing
      System.out.println("Actual End Ang: " + driveTrain.getGyroRaw()); // can be removed after testing
      return true;
    }
    return false;
  }
}