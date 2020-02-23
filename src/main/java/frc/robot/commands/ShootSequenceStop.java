/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class ShootSequenceStop extends SequentialCommandGroup {
  /**
   * Stop feeder, hopper, and intake motors. Set shooter to low RPM.
   * @param shooter shooter subsystem
   * @param feeder feeder subsystem
   * @param hopper hopper subsystem
   * @param intake intake subsystem
   * @param led led strip
   */
  public ShootSequenceStop(Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {
    addCommands(
      parallel(
        new ShooterSetPID(1200, shooter, led),
        new FeederSetVoltage(0, feeder),
        new IntakeSetPercentOutput(0, intake),
        new HopperSetPercentOutput(0, hopper)
      )
    );
  }
}
