/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Command group to run the shooter, feeder, intake, and hopper for shooting.
 */
public class ShooterFeederHopperSequence extends SequentialCommandGroup {
  /**
   * Setpoint rpm for shooter is set from dashboard.
   * @param shooter shooter subsystem to use
   * @param feeder feeder subsystem to use
   * @param hopper hopper subsystem to use
   * @param intake intake subsystem to use
   */
  public ShooterFeederHopperSequence(Shooter shooter, Feeder feeder, Hopper hopper, Intake intake) {
    if(SmartDashboard.getNumber("Shooter Manual SetPoint RPM", -9999) == -9999)
      SmartDashboard.putNumber("Shooter Manual SetPoint RPM", 2800);
    double rpm = SmartDashboard.getNumber("Shooter Manual SetPoint RPM", 2800);

    addCommands( 
      new ShooterSetPID(rpm, shooter),
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
  public ShooterFeederHopperSequence(int rpm, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake) {
    addCommands( 
      new ShooterSetPID(rpm, shooter),
      new FeederSetPID(feeder),
      new HopperSetPercentOutput(hopper),
      new ParallelCommandGroup(new IntakeSetPercentOutput(intake), new HopperReverse(hopper))
    );
  }
}
