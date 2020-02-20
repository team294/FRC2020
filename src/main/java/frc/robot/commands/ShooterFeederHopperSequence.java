/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LED;

/**
 * Command group to run the shooter, feeder, intake, and hopper for shooting.
 */
public class ShooterFeederHopperSequence extends SequentialCommandGroup {
  /**
   * @param rpmFromDistance true = rpm is set with distance from target, false = rpm is set with manual dashboard input
   * @param shooter shooter subsystem to use
   * @param feeder feeder subsystem to use
   * @param hopper hopper subsystem to use
   * @param intake intake subsystem to use
   */

  public ShooterFeederHopperSequence(boolean rpmFromDistance, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {
    addCommands( 
      new ShooterSetPID(rpmFromDistance, shooter, led),
      new FeederSetPID(feeder),
      new HopperSetPercentOutput(hopper),
      new ParallelCommandGroup(new IntakeSetPercentOutput(intake), new HopperReverse(hopper))
    );
  }

  /**
   * @param rpm setpoint rpm for shooter
   * @param shooter shooter subsystem to use
   * @param feeder feeder subsystem to use
   * @param hopper hopper subsystem to use
   * @param intake intake subsystem to use
   */
  public ShooterFeederHopperSequence(int rpm, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {
    addCommands( 
      new ShooterSetPID(rpm, shooter, led),
      new FeederSetPID(feeder),
      new HopperSetPercentOutput(-1 * HopperConstants.hopperDefaultPercentOutput, hopper),
      new ParallelCommandGroup(new IntakeSetPercentOutput(intake), new HopperReverse(hopper))
    );
  }
}
