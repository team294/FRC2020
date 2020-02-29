/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class ShootSequence extends SequentialCommandGroup {
  /**
   * Set shooter to setpoint RPM using either distance to target or dashboard input. Then run the feeder, intake, and hopper.
   * @param rpmFromDistance true = rpm using distance from target, false = rpm using dashboard input
   * @param shooter shooter subsystem
   * @param feeder feeder subsystem
   * @param hopper hopper subsystem
   * @param intake intake subsystem
   * @param limeLight limelight camera (subsystem)
   * @param led led strip (subsystem)
   */
  public ShootSequence(boolean rpmFromDistance, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LimeLight limeLight, LED led, FileLog log) {
    addCommands(
      parallel(
        // If getting RPM from distance and within range to do unlocked hood shot,
        // unlock the hood but close it. Otherwise, close and lock the hood.
        new ConditionalCommand(
          new ShooterHoodPistonSequence(true, false, shooter, log),
          new ShooterHoodPistonSequence(true, true, shooter, log),
          () -> rpmFromDistance && limeLight.getDistanceNew() < LimeLightConstants.unlockedHoodMaxDistance
        )
      ),
      new ShooterSetPID(rpmFromDistance, true, shooter, limeLight, led, log),
      new FeederSetPID(FeederConstants.feederDefaultRPM, feeder, log),
      new HopperSetPercentOutput(-1 * HopperConstants.hopperDefaultPercentOutput, true, hopper, log),
      parallel(
        new IntakeSetPercentOutput(false, intake, log), 
        new HopperReverse(hopper, log)
      )
    );
  }

 /**
  * Set shooter to setpoint RPM using parameter RPM. Then run the feeder, intake, and hopper.
   * @param rpm setpoint rpm for shooter
   * @param shooter shooter subsystem
   * @param feeder feeder subsystem
   * @param hopper hopper subsystem
   * @param intake intake subsystem
   * @param led led strip
  * @param log
  */
  public ShootSequence(int rpm, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led, FileLog log) {
    addCommands( 
      new ShooterSetPID(rpm, shooter, led, log),
      new FeederSetPID(feeder, log),
      new HopperSetPercentOutput(-1 * HopperConstants.hopperDefaultPercentOutput, true, hopper, log),
      parallel(
        new IntakeSetPercentOutput(false, intake, log), 
        new HopperReverse(hopper, log)
      )
    );
  }

 /**
  * Set shooter to setpoint RPM for short shot. Then run the feeder, intake, and hopper.
  * @param shooter shooter subsystem
  * @param feeder feeder subsystem
  * @param hopper hopper subsystem
  * @param intake intake subsystem
  * @param limeLight limelight camera (subsystem)
  * @param led led strip (subsystem)
  * @param log Filelog subsystem
  */
  public ShootSequence(Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LimeLight limeLight, LED led, FileLog log) {
    addCommands( 
      parallel(
        // If hood is not open, wait 0.3 seconds before moving on from setting hood position.
        // Otherwise, immediately move on from setting hood position.
        new ConditionalCommand(new Wait(0.3), new Wait(0), () -> !shooter.getHoodPiston()),
        new ShooterHoodPistonSequence(false, false, shooter, log) 
      ),
      new ConditionalCommand(
        new ShooterSetPID(ShooterConstants.shooterDefaultShortRPM, shooter, led, log),
        new ShooterSetPID(true, true, shooter, limeLight, led, log),
        () -> limeLight.getDistanceNew() != 0
      ),
      new FeederSetPID(feeder, log),
      new HopperSetPercentOutput(-1 * HopperConstants.hopperDefaultPercentOutput, true, hopper, log),
      parallel(
        new IntakeSetPercentOutput(false, intake, log), 
        new HopperReverse(hopper, log)
      )
    );
  }
}
