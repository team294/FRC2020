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

public class DriveStraightRegenerate extends CommandBase {
  /**
   * Uses wpilib TrapezoidProfile generator to generate a motion profile for drive train turning
   */

  private DriveTrain driveTrain; // reference to driveTrain
  private FileLog log;

  private double target; // how many more degrees to the right to turn
  private double maxVelMultiplier; // multiplier between 0.0 and 1.0 for limiting max velocity
  private double maxAccelMultiplier; // multiplier between 0.0 and 1.0 for limiting max acceleration
  private long profileStartTime; // initial time (time of starting point)
  private long currProfileTime;
  private double targetVel; // velocity to reach by the end of the profile in deg/sec (probably 0 deg/sec)
  private double targetAccel;
  private double targetVoltage; // voltage to pass to the motors to follow profile
  private double startDist; // initial angle (in degrees) (starts as 0 deg for relative turns)
  private double currDist;
  private double linearVel;
  private double endTime;
  private double timeSinceStart;

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
  public DriveStraightRegenerate(DriveTrain driveTrain, FileLog log, double target, double maxVelMultiplier, double maxAccelMultiplier) {
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
    tStateFinal = new TrapezoidProfileBCR.State(target, 0.0); // initialize goal state (degrees to turn)
    tStateCurr = new TrapezoidProfileBCR.State(0.0, 0.0); // initialize initial state (relative turning, so assume initPos is 0 degrees)

    tConstraints = new TrapezoidProfileBCR.Constraints(kMaxSpeedMetersPerSecond * maxVelMultiplier, kMaxAccelerationMetersPerSecondSquared * maxAccelMultiplier); // initialize velocity
                                                                                                                          // and accel limits
    tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr); // generate profile
    System.out.println(tProfile.totalTime());
    
    endTime = tProfile.totalTime();
    profileStartTime = System.currentTimeMillis(); // save starting time of profile
    currProfileTime = profileStartTime;
    startDist = Units.inchesToMeters(driveTrain.getAverageDistance());
    // prevAng = startAng;
    // targetAng = startAng + target; // calculate final angle based on how many degrees are to be turned
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.feedTheDog();
    currProfileTime = System.currentTimeMillis();
    timeSinceStart = (double)(currProfileTime - profileStartTime) / 1000.0;
    tStateNext = tProfile.calculate(timeSinceStart);

    targetVel = tStateNext.velocity;
    targetAccel = tStateNext.acceleration;
    targetVoltage = (kSLinear * Math.signum(targetVel)) + (targetVel * kVLinear) + (targetAccel * kALinear);

    SmartDashboard.putNumber("pos: ", tStateNext.position);
    SmartDashboard.putNumber("vel: ", targetVel);
    SmartDashboard.putNumber("V: ", targetVoltage);

    log.writeLog(false, "DriveStraight", "profile", "posT", tStateNext.position, "velT", targetVel, 
    "posA", (Units.inchesToMeters(driveTrain.getAverageDistance()) - startDist), 
    "velLA", (driveTrain.getLeftEncoderVelocity()*2.52 / 100), "velRA", (driveTrain.getRightEncoderVelocity()*2.52 / 100), "V", targetVoltage);
    System.out.println("pos: " + tStateNext.position);
    System.out.println("vel: " + targetVel);
    System.out.println("V: " + targetVoltage);

    driveTrain.setRightMotorOutput(-targetVoltage);
    driveTrain.setLeftMotorOutput(targetVoltage);

    currDist = Units.inchesToMeters(driveTrain.getAverageDistance()) - startDist;
    linearVel = driveTrain.getAverageEncoderVelocity() * 2.54 / 100;
    tStateCurr = new TrapezoidProfileBCR.State(currDist, linearVel);
    tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr);
    profileStartTime = currProfileTime;
    endTime = tProfile.totalTime();
    System.out.println(endTime);

    driveTrain.feedTheDog();
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
      System.out.println("Start: " + startDist);
      System.out.println("theoretical: " + (startDist + target));
      System.out.println("actual: " + Units.inchesToMeters(driveTrain.getAverageDistance()));
      return true;
    }
    return false;
  }
}