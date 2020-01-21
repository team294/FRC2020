/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;


import frc.robot.subsystems.*;
import static frc.robot.Constants.DriveConstants.*;

public class DriveTurnGyro extends CommandBase {
  /**
   * Uses wpilib TrapezoidProfile generator to generate a motion profile for drive train turning
   */

  private DriveTrain driveTrain; // reference to driveTrain
  private double target; // how many more degrees to the right to turn
  private double maxVelMultiplier; // multiplier between 0.0 and 1.0 for limiting max velocity
  private double maxAccelMultiplier; // multiplier between 0.0 and 1.0 for limiting max acceleration
  private double profileStartTime; // initial time (time of starting point)
  private double targetVel; // velocity to reach by the end of the profile in deg/sec (probably 0 deg/sec)
  private double targetVoltage; // voltage to pass to the motors to follow profile
  private double startAng; // initial angle (in degrees) (starts as 0 deg for relative turns)
  private double targetAng; // target angle to the right (in degrees)

  private TrapezoidProfile tProfile; // wpilib trapezoid profile generator
  private State tStateInit; // initial state of the system (position in deg and time in sec)
  private State tStateNext; // next state of the system as calculated by the profile generator
  private State tStateFinal; // goal state of the system (position in deg and time in sec)
  private Constraints tConstraints; // max vel (deg/sec) and max accel (deg/sec/sec) of the system

  /**
   * @param driveTrain reference to the drive train subsystem
   * @param target degrees to turn to the right
   * @param maxVelMultiplier between 0.0 and 1.0, multipier for limiting max velocity
   * @param maxAccelMultiplier between 0.0 and 1.0, multiplier for limiting max acceleration
   */
  public DriveTurnGyro(DriveTrain driveTrain, double target, double maxVelMultiplier, double maxAccelMultiplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.target = target;
    this.maxVelMultiplier = maxVelMultiplier;
    this.maxAccelMultiplier = maxAccelMultiplier;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tStateFinal = new State(target, 0); // initialize goal state (degrees to turn)
    tStateInit = new State(0, 0); // initialize initial state (relative turning, so assume initPos is 0 degrees)
    tConstraints = new Constraints(kMaxAngularVelocity * maxVelMultiplier, kMaxAngularAcceleration * maxAccelMultiplier); // initialize velocity
                                                                                                                          // and accel limits
    tProfile = new TrapezoidProfile(tConstraints, tStateFinal, tStateInit); // generate profile
    System.out.println(tProfile.totalTime());

    profileStartTime = System.currentTimeMillis() / 1000; // save starting time of profile
    startAng = driveTrain.getGyroRotation(); // save initial angle
    targetAng = startAng + target; // calculate final angle based on how many degrees are to be turned
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tStateNext = tProfile.calculate((System.currentTimeMillis() / 1000) - profileStartTime);
    targetVel = tStateNext.velocity;
    targetVoltage = targetVel * kVAngular;

    System.out.println("pos: " + tStateNext.position);
    System.out.println("vel: " + targetVel);
    System.out.println("V: " + targetVoltage);
    driveTrain.setRightMotorOutput(targetVoltage);
    driveTrain.setLeftMotorOutput(targetVoltage);
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
    if(Math.abs(targetAng - startAng) < 1) {
      return true;
    }
    return false;
  }
}