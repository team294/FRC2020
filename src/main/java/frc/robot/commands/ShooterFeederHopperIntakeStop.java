/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

/**
 * Command group to stop the shooter, feeder, hopper, and intake.
 * NOTE: THIS SEQUENCE NOW SETS THE SHOOTER TO 1200 RPM
 */
public class ShooterFeederHopperIntakeStop extends SequentialCommandGroup {
  /**
   * @param shooter shooter subsystem to use
   * @param feeder feeder subsystem to use
   * @param hopper hopper subsystem to use
   * @param intake intake subsystem to use
   * @param led led subsystem to use
   */
  public ShooterFeederHopperIntakeStop(Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {
    addCommands(
      //new FeederSetPiston(false, feeder),
      parallel(
        // new ShooterSetVoltage(0, shooter),
        new ShooterSetPID(1200, shooter, led),
        new FeederSetVoltage(0, feeder),
        new IntakeSetPercentOutput(0, intake),
        new HopperSetPercentOutput(0, hopper)
      )
    );
  }
}
