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
  public ShootSequence(boolean rpmFromDistance, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LimeLight limeLight, LED led) {
    addCommands(
      parallel(
        // If getting RPM from distance and within range to do unlocked hood shot,
        // unlock the hood but close it. Otherwise, close and lock the hood.
        new ConditionalCommand(
          new ShooterHoodPistonSequence(true, true, shooter),
          new ShooterHoodPistonSequence(true, false, shooter),
          () -> rpmFromDistance && (limeLight.getDistanceNew() > LimeLightConstants.unlockedHoodMaxDistance
            || !limeLight.seesTarget())
        )
      ),
      new ShooterSetPID(rpmFromDistance, true, shooter, limeLight, led),
      new FeederSetPID(FeederConstants.feederDefaultRPM, feeder),
      new HopperSetPercentOutput(-1 * HopperConstants.hopperDefaultPercentOutput, true, hopper),
      parallel(
        new IntakeSetPercentOutput(false, intake), 
        new HopperReverse(hopper)
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
   */
  public ShootSequence(int rpm, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {
    addCommands( 
      new ShooterSetPID(rpm, shooter, led),
      new FeederSetPID(feeder),
      new HopperSetPercentOutput(-1 * HopperConstants.hopperDefaultPercentOutput, true, hopper),
      parallel(
        new IntakeSetPercentOutput(false, intake), 
        new HopperReverse(hopper)
      )
    );
  }

  /**
   * Set shooter to setpoint RPM using default values for shooting from the trench
   * or the auto line. Then run the feeder, intake, and hopper.
   * This constructor is used for forcing the shooter to a certain position and RPM,
   * disregarding what vision is tracking.
   * @param trench true = shooting from trench, false = shooting from auto line
   * @param shooter shooter subsystem
   * @param feeder feeder subsystem
   * @param hopper hopper subsystem
   * @param intake intake subsystem
   * @param led led strip
   */
  public ShootSequence(boolean trench, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {
    addCommands( 
      // If shooting from the trench, close the hood, lock it, and set shooter RPM. 
      // Otherwise, close the hood, leave it unlocked, and set shooter RPM.
      new ConditionalCommand(
        sequence(
          new ShooterHoodPistonSequence(true, true, shooter),
          new ShooterSetPID(ShooterConstants.shooterDefaultTrenchRPM, shooter, led)
        ),
        sequence(
          new ShooterHoodPistonSequence(true, false, shooter),
          new ShooterSetPID(ShooterConstants.shooterDefaultRPM, shooter, led)
        ),
        () -> trench
      ),
      new FeederSetPID(feeder),
      new HopperSetPercentOutput(-1 * HopperConstants.hopperDefaultPercentOutput, true, hopper),
      parallel(
        new IntakeSetPercentOutput(false, intake), 
        new HopperReverse(hopper)
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
   */
  public ShootSequence(Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LimeLight limeLight, LED led) {
    addCommands( 
      parallel(
        // If hood is not open, wait 0.3 seconds before moving on from setting hood position.
        // Otherwise, immediately move on from setting hood position.
        new ConditionalCommand(new Wait(0.3), new Wait(0), () -> !shooter.getHoodPiston()),
        new ShooterHoodPistonSequence(false, false, shooter) 
      ),
      new ConditionalCommand(
        new ShooterSetPID(ShooterConstants.shooterDefaultShortRPM, shooter, led),
        new ShooterSetPID(true, true, shooter, limeLight, led),
        () -> limeLight.getDistanceNew() != 0
      ),
      new FeederSetPID(feeder),
      new HopperSetPercentOutput(-1 * HopperConstants.hopperDefaultPercentOutput, true, hopper),
      parallel(
        new IntakeSetPercentOutput(false, intake), 
        new HopperReverse(hopper)
      )
    );
  }
}
