/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.*;

public class ShootSequence extends SequentialCommandGroup {
  /**
   * Set shooter to setpoint RPM using either distance to target or dashboard input. Then run the feeder, intake, and hopper.
   * @param rpmFromDistance true = rpm using distance from target, false = rpm using dashboard input
   * @param shooter shooter subsystem
   * @param feeder feeder subsystem
   * @param hopper hopper subsystem
   * @param intake intake subsystem
   * @param limeLight limelight camera
   * @param led led strip
   */
  public ShootSequence(boolean rpmFromDistance, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LimeLight limeLight, LED led) {
    addCommands( 
      new ShooterSetPID(rpmFromDistance, shooter, limeLight, led),
      new FeederSetPID(feeder),
      new HopperSetPercentOutput(-1 * Constants.HopperConstants.hopperDefaultPercentOutput, hopper),
      parallel(
        new IntakeSetPercentOutput(intake), 
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
      new HopperSetPercentOutput(-1 * HopperConstants.hopperDefaultPercentOutput, hopper),
      parallel(
        new IntakeSetPercentOutput(intake), 
        new HopperReverse(hopper)
      )
    );
  }
}
