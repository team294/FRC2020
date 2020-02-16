/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;

/**
 * Command group to deploy the intake and run the rollers.
 */
public class IntakeSequence extends SequentialCommandGroup {
  /**
   * @param intake intake subsystem to use
   */
  public IntakeSequence(Intake intake) {
    addCommands(
      new IntakePistonSetPosition(true, intake),
      new IntakeSetPercentOutput(intake)
    );
  }
}
